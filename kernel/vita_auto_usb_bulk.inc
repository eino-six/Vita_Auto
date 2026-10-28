// Bulk memory and session bookkeeping.
static int allocateBulkMemory(void)
{
    void *base = NULL;
    int result;

    if (g_bulkDmaMemBlock >= 0)
        return 0;

    g_bulkDmaMemBlock = ksceKernelAllocMemBlock(
        "vita_auto_usb_bulk",
        SCE_KERNEL_MEMBLOCK_TYPE_KERNEL_ROOT_PHYCONT_NC_RW,
        AA_BULK_DMA_BLOCK_SIZE,
        NULL
    );
    if (g_bulkDmaMemBlock < 0)
        return g_bulkDmaMemBlock;

    result = ksceKernelGetMemBlockBase(g_bulkDmaMemBlock, &base);
    if (result < 0) {
        ksceKernelFreeMemBlock(g_bulkDmaMemBlock);
        g_bulkDmaMemBlock = -1;
        return result;
    }

    g_bulkShared = (UsbBulkShared *)base;
    g_bulkReadRing = (unsigned char (*)[AA_BULK_READ_SLOT_SIZE])(
        (unsigned char *)base + AA_BULK_SHARED_BYTES
    );
    g_bulkReadDmaBuffers =
        (unsigned char (*)[AA_BULK_READ_SLOT_SIZE])(
            (unsigned char *)g_bulkReadRing + AA_BULK_READ_RING_BYTES
        );
    g_bulkWriteBuffer =
        (unsigned char *)g_bulkReadDmaBuffers + AA_BULK_READ_DMA_BYTES;
    return 0;
}

static int freeBulkMemory(void)
{
    if (g_bulkDmaMemBlock >= 0) {
        const int result = ksceKernelFreeMemBlock(g_bulkDmaMemBlock);
        if (result < 0)
            return result;
    }
    g_bulkDmaMemBlock = -1;
    g_bulkShared = NULL;
    g_bulkReadRing = NULL;
    g_bulkReadDmaBuffers = NULL;
    g_bulkWriteBuffer = NULL;
    return 0;
}

// FNV-1a detects corruption between publication and consumption.
static uint32_t checksumBytes(const unsigned char *data, int length)
{
    uint32_t hash = 2166136261u;
    int index;

    for (index = 0; index < length; ++index) {
        hash ^= data[index];
        hash *= 16777619u;
    }
    return hash;
}

static uint32_t bulkReadRingDepth(int head, int tail)
{
    return head >= tail
        ? (uint32_t)(head - tail)
        : (uint32_t)(AA_BULK_READ_SLOT_COUNT - tail + head);
}

static void clearBulkReadSlot(int slot)
{
    g_bulkShared->readLength[slot] = 0;
    g_bulkShared->readChecksum[slot] = 0;
    g_bulkShared->readSequence[slot] = 0;
    g_bulkShared->readCompletedAtUs[slot] = 0;
}

// Reset session counters and request slots when starting or retiring I/O.
static void resetBulkTransfers(void)
{
    int i;

    for (i = 0; i < AA_BULK_READ_SLOT_COUNT; ++i)
        clearBulkReadSlot(i);

    g_bulkShared->readHead = 0;
    g_bulkShared->readTail = 0;
    g_bulkShared->readSubmitCount = 0;
    g_bulkShared->readCompleteCount = 0;
    g_bulkShared->readPublishedCount = 0;
    g_bulkShared->readConsumedCount = 0;
    g_bulkShared->readChecksumFailures = 0;
    g_bulkShared->readSequenceFailures = 0;
    g_bulkShared->readMaxDepth = 0;
    g_bulkShared->readNextSequence = 1;
    g_bulkShared->readExpectedSequence = 1;
    g_bulkReadSubmitSequence = 1;
    g_bulkReadPublishSequence = 1;
    g_bulkReadSchedulerBusy = 0;
    g_bulkReadSchedulerPending = 0;
    g_bulkReadsDraining = 0;
    g_bulkReadConcurrencyLimit = AA_BULK_READ_PIPELINE_DEPTH;
    g_bulkReadTransientFailures = 0;
    g_bulkShared->readArmed = 0;
    for (i = 0; i < AA_BULK_READ_PIPELINE_DEPTH; ++i) {
        g_bulkReadRequests[i].state = BULK_READ_FREE;
        g_bulkReadRequests[i].submitSequence = 0;
        g_bulkReadRequests[i].length = 0;
        g_bulkReadRequests[i].completedAtUs = 0;
        g_bulkReadRequests[i].buffer = g_bulkReadDmaBuffers != NULL
            ? g_bulkReadDmaBuffers[i]
            : NULL;
    }
    g_bulkShared->writeBusy = 0;
    g_bulkShared->writeSubmitCount = 0;
    g_bulkShared->writeCompleteCount = 0;
    g_bulkShared->writeFailureCount = 0;
    g_bulkShared->writeExpectedLength = 0;
    g_bulkShared->writeLastResult = 0;
    g_bulkShared->writeLastBytes = 0;
}

static void setUsbFailure(enum VitaAutoUsbFailureStage stage, int result, int count)
{
    if (!sessionIsOpen())
        return; /* Stop does not mean failure */
    g_lastFailureStage = stage;
    g_lastFailureResult = result;
    g_lastFailureCount = count;
    retireSession(USB_FAILED);
}

/*
 USB has no standard pre-accessory USB vendor ID. Keep the first hardware
 common Android vendors and avoid taking over arbitrary USB devices.
 */
static int isSupportedAndroidVendor(unsigned short vendorId)
{
    switch (vendorId) {
    case 0x18d1: /* Google / Android accessory mode */
    case 0x04e8: /* Samsung */
    case 0x22b8: /* Motorola */
    case 0x2a70: /* OnePlus */
    case 0x2717: /* Xiaomi */
        return 1;
    default:
        return 0;
    }
}

static int isAccessoryDevice(const SceUsbdDeviceDescriptor *device)
{
    return device->idVendor == GOOGLE_VENDOR_ID &&
        (device->idProduct == USB_ACCESSORY_PID ||
         device->idProduct == USB_ACCESSORY_AUDIO_PID);
}

static int submitBulkRead(void);

static void handleUsbTransferDone(int32_t result, int32_t count, void *arg);
static void handleAccessoryConfigurationDone(int32_t result, int32_t count, void *arg);
static void handleAaBulkReadDone(int32_t result, int32_t count, void *arg);
static void handleAaBulkWriteDone(int32_t result, int32_t count, void *arg);

/* A detach can retire a pipe without completing its queued URBs. Stop clears
   the accounting for those stranded requests, so a late completion from the
   old session must not underflow the counter used by the next session. */
static void releasePendingTransfer(void)
{
    int current;
    do {
        current = __sync_fetch_and_add(&g_pendingTransfers, 0);
        if (current <= 0)
            return;
    } while (!__sync_bool_compare_and_swap(
        &g_pendingTransfers, current, current - 1));
}

static void completeTransfer(
    ksceUsbdDoneCallback callback, int32_t result, int32_t count, void *arg)
{
    if (enterOperation(0)) {
        callback(result, count, arg);
        leaveOperation();
    }
    releasePendingTransfer();
}

static void usbTransferDone(int32_t result, int32_t count, void *arg)
{
    // A late completion from the original identity must not fail its successor.
    if (g_normalDeviceId >= 0 && g_accessoryDeviceId < 0)
        completeTransfer(handleUsbTransferDone, result, count, arg);
    else
        releasePendingTransfer();
}

static void accessoryConfigurationDone(int32_t result, int32_t count, void *arg)
{
    completeTransfer(handleAccessoryConfigurationDone, result, count, arg);
}

static void aaBulkReadDone(int32_t result, int32_t count, void *arg)
{
    completeTransfer(handleAaBulkReadDone, result, count, arg);
}

static void aaBulkWriteDone(int32_t result, int32_t count, void *arg)
{
    completeTransfer(handleAaBulkWriteDone, result, count, arg);
}

static int queueControlTransfer(
    SceUID pipe, const SceUsbdDeviceRequest *request, unsigned char *buffer,
    ksceUsbdDoneCallback callback, void *arg)
{
    int result;
    if (!sessionIsOpen())
        return -1;
    __sync_add_and_fetch(&g_pendingTransfers, 1);
    result = ksceUsbdControlTransfer(pipe, request, buffer, callback, arg);
    if (result < 0)
        releasePendingTransfer();
    return result;
}

static int queueBulkTransfer(
    SceUID pipe, unsigned char *buffer, uint32_t length,
    ksceUsbdDoneCallback callback, void *arg)
{
    int result;
    if (!sessionIsOpen())
        return -1;
    __sync_add_and_fetch(&g_pendingTransfers, 1);
    result = ksceUsbdBulkTransfer(pipe, buffer, length, callback, arg);
    if (result < 0)
        releasePendingTransfer();
    return result;
}

// Android Open Accessory negotiation.
static int submitControlRequest(
    unsigned char requestType,
    unsigned char request,
    unsigned short value,
    unsigned short index,
    unsigned short length,
    unsigned char *data
) {
    g_usb.request.bmRequestType = requestType;
    g_usb.request.bRequest = request;
    g_usb.request.wValue = value;
    g_usb.request.wIndex = index;
    g_usb.request.wLength = length;

    return queueControlTransfer(
        g_usb.controlPipe,
        &g_usb.request,
        data,
        usbTransferDone,
        NULL
    );
}

static int submitStart(void)
{
    g_usb.state = USB_STARTING;
    return submitControlRequest(
        USB_REQ_TO_DEVICE_VENDOR,
        USB_START,
        0,
        0,
        0,
        NULL
    );
}

static int submitNextString(void)
{
    static const char *const strings[] = {
        "Android",
        "Android Auto",
        "Android Auto",
        "1.0",
        "https://www.android.com/auto/",
        "VITAAUTO01"
    };

    const char *value;
    unsigned int length = 0;

    if (g_usb.stringIndex >= (sizeof(strings) / sizeof(strings[0])))
        return submitStart();

    value = strings[g_usb.stringIndex];

    while (value[length] != '\0')
        ++length;

    if (length + 1 > sizeof(g_controlBuffer))
        return (int)SCE_USBD_ERROR_INVALID_PARAM;

    {
        unsigned int i;
        for (i = 0; i <= length; ++i)
            g_controlBuffer[i] = (unsigned char)value[i];
    }

    return submitControlRequest(
        USB_REQ_TO_DEVICE_VENDOR,
        USB_SEND_STRING,
        0,
        (unsigned short)g_usb.stringIndex,
        (unsigned short)(length + 1),
        g_controlBuffer
    );
}

static void handleUsbTransferDone(int32_t result, int32_t count, void *arg)
{
    (void)arg;

    if (result < 0) {
        /*
         A control completion can arrive after the accessory has already
         attached, or after the normal device detached while START was in
         flight. Its old normal-device pipe is irrelevant then; do not turn
         it into a session failure.
         */
        if (g_accessoryDeviceId >= 0 ||
            g_usb.state == USB_IDLE ||
            g_usb.state == USB_WAITING_FOR_ACCESSORY)
            return;

        /*
         The phone may abort the START control transfer as it switches to
         accessory mode. Set the waiting flag before retrying so an accessory
         that re-enumerates during a retry is accepted instead of rejected.
         START is special: the abort can be the switch itself, not a request
         that needs to be sent again.
         */
        if (g_usb.state == USB_STARTING &&
            result == (int)SCE_USBD_ERROR_8024000A) {
            g_waitingForAccessory = 1;
            if (g_aoaRetryCount < AA_AOA_MAX_RETRIES) {
                ++g_aoaRetryCount;
                g_aoaRetryPolls = AA_AOA_RETRY_POLLS;
                g_aoaRetryPending = 1;
                g_aoaRetryState = USB_STARTING;
                ksceDebugPrintf(
                    "[VitaAutoUSB] START interrupted; waiting for accessory retry=%d\n",
                    g_aoaRetryCount
                );
                return;
            }
            g_aoaRetryPending = 0;
            g_aoaRetryCount = 0;
            g_usb.state = USB_WAITING_FOR_ACCESSORY;
            ksceDebugPrintf(
                "[VitaAutoUSB] START retries exhausted; waiting for accessory\n"
            );
            return;
        }

        /*
         The phone can interrupt any AOA control request while its gadget
         settles after enumeration. Retry GET_PROTOCOL, SEND_STRING, and START
         with the same transient policy used for AA version bulk writes.
         */
        if ((g_usb.state == USB_GETTING_PROTOCOL ||
             g_usb.state == USB_SENDING_STRINGS ||
             g_usb.state == USB_STARTING) &&
            result == (int)SCE_USBD_ERROR_8024000A &&
            g_aoaRetryCount < AA_AOA_MAX_RETRIES) {
            ++g_aoaRetryCount;
            g_aoaRetryPolls = AA_AOA_RETRY_POLLS;
            g_aoaRetryPending = 1;
            g_aoaRetryState = g_usb.state;
            g_aoaRetryStringIndex = g_usb.stringIndex;
            ksceDebugPrintf(
                "[VitaAutoUSB] AOA request interrupted; state=%d index=%u retry=%d\n",
                g_usb.state,
                g_usb.stringIndex,
                g_aoaRetryCount
            );
            return;
        }

        ksceDebugPrintf("[VitaAutoUSB] USB request failed: 0x%08x\n", result);
        {
            int failureCount = count;

            if (g_usb.state == USB_GETTING_PROTOCOL)
                failureCount = g_aoaRetryCount;
            else if (g_usb.state == USB_SENDING_STRINGS)
                failureCount = 0x100 | (int)(g_usb.stringIndex & 0xffu);
            else if (g_usb.state == USB_STARTING)
                failureCount = 0x200 | (g_aoaRetryCount & 0xff);

            setUsbFailure(
                VITA_AUTO_USB_FAILURE_CONTROL_TRANSFER,
                result,
                failureCount
            );
        }
        return;
    }

    if (g_usb.state == USB_GETTING_PROTOCOL) {
        unsigned int protocolVersion;

        if (count != 2) {
            ksceDebugPrintf("[VitaAutoUSB] GET_PROTOCOL returned %d bytes\n", count);
            setUsbFailure(VITA_AUTO_USB_FAILURE_PROTOCOL_LENGTH, result, count);
            return;
        }

        protocolVersion = g_usb.protocol[0] | (g_usb.protocol[1] << 8);

        if (protocolVersion == 0) {
            ksceDebugPrintf("[VitaAutoUSB] Device does not support USB\n");
            setUsbFailure(VITA_AUTO_USB_FAILURE_PROTOCOL_UNSUPPORTED, 0, count);
            return;
        }

        vitaUsbLogInfo("[VitaAutoUSB] USB protocol %u detected\n", protocolVersion);
        g_usb.state = USB_SENDING_STRINGS;
        g_usb.stringIndex = 0;
        g_aoaRetryCount = 0;

        {
            const int submitResult = submitNextString();
            if (submitResult < 0)
                setUsbFailure(VITA_AUTO_USB_FAILURE_FIRST_ACCESSORY_STRING, submitResult, 0);
        }
        return;
    }

    if (g_usb.state == USB_SENDING_STRINGS) {
        ++g_usb.stringIndex;
        g_aoaRetryCount = 0;

        {
            const int submitResult = submitNextString();
            if (submitResult < 0)
                setUsbFailure(VITA_AUTO_USB_FAILURE_NEXT_ACCESSORY_REQUEST, submitResult, 0);
        }
        return;
    }

    if (g_usb.state == USB_STARTING) {
        // The phone now re-enumerates as Google VID 18d1 / PID 2d00 or 2d01.
        vitaUsbLogInfo("[VitaAutoUSB] USB start sent; waiting for phone re-enumeration\n");
        g_waitingForAccessory = 1;
        g_usb.state = USB_WAITING_FOR_ACCESSORY;
        g_aoaRetryCount = 0;
    }
}

// Bulk transfer completions and receive scheduler.
static void handleAaBulkWriteDone(int32_t result, int32_t count, void *arg)
{
    /* Capture before releasing writeBusy. The app may immediately queue its
     next write and replace writeExpectedLength on another core. */
    const int writeFailed = result < 0 || count < 0 ||
        count != (int)g_bulkShared->writeExpectedLength;
    (void)arg;

    ++g_bulkShared->writeCompleteCount;
    g_bulkShared->writeLastResult = result;
    g_bulkShared->writeLastBytes = count;
    if (writeFailed) {
        ++g_bulkShared->writeFailureCount;
    }
    g_bulkShared->writeBusy = 0;

    if (writeFailed) {
        if (g_usb.state == USB_AA_VERSION_SENT &&
            result == (int)SCE_USBD_ERROR_8024000A &&
            g_accessoryDeviceId >= 0 &&
            g_usb.bulkOutPipe >= 0 &&
            g_versionRetryCount < AA_VERSION_MAX_RETRIES) {
            ++g_versionRetryCount;
            g_versionSettlePolls = AA_VERSION_RETRY_POLLS;
            g_versionWriteCompleted = 0;
            g_usb.state = USB_OPENING_BULK;
            ksceDebugPrintf(
                "[VitaAutoUSB] AA version write transient; retry %d\n",
                g_versionRetryCount
            );
            return;
        }

        ksceDebugPrintf(
            "[VitaAutoUSB] Bulk write failed: result=0x%08x count=%d\n",
            result,
            count
        );
        setUsbFailure(VITA_AUTO_USB_FAILURE_BULK_WRITE, result, count);
        return;
    }

    if (g_usb.state == USB_AA_VERSION_SENT) {
        g_versionWriteCompleted = 1;
        vitaUsbLogInfo("[VitaAutoUSB] Android Auto 1.7 version request sent\n");
    }
}

// Publish request fields before the scheduler sees COMPLETE.
static void completeBulkReadRequest(BulkReadRequest *request, int length)
{
    request->length = length;
    __sync_synchronize();
    request->state = BULK_READ_COMPLETE;
}

static void handleAaBulkReadDone(int32_t result, int32_t count, void *arg)
{
    BulkReadRequest *request = (BulkReadRequest *)arg;

    if (request == NULL)
        return;

    ++g_bulkShared->readCompleteCount;
    request->completedAtUs = ksceKernelGetSystemTimeWide();
    if (g_bulkShared->readArmed > 0)
        __sync_sub_and_fetch(&g_bulkShared->readArmed, 1);

    if (result < 0) {
        ksceDebugPrintf(
            "[VitaAutoUSB] AA bulk read failed: 0x%08x count=%d state=%d armed=%d submit=%u complete=%u\n",
            (unsigned int)result,
            count,
            g_usb.state,
            g_bulkShared->readArmed,
            g_bulkShared->readSubmitCount,
            g_bulkShared->readCompleteCount
        );

        /*
         0x8024000A is the transient controller error already tolerated for
         AOA control requests. The controller can report it after it has
         already moved bytes; the v10 build retired that request as SKIPPED
         and dropped the bytes, leaving a hole in the AA byte stream. TLS
         then failed with INVALID_MAC on a record whose sequence number was
         correct, or the outer AA parser lost framing, depending on where the
         hole landed. Publish what the controller reports, exactly like a
         short read, so the stream stays contiguous.
        */
        if (result == (int)SCE_USBD_ERROR_8024000A &&
            count > 0 &&
            count <= AA_BULK_READ_SLOT_SIZE &&
            g_usb.state == USB_AA_VERSION_COMPLETE &&
            g_bulkReadTransientFailures < AA_BULK_READ_MAX_TRANSIENT &&
            sessionIsOpen()) {
            ++g_bulkReadTransientFailures;
            ++g_bulkShared->readChecksumFailures;
            ksceDebugPrintf(
                "[VitaAutoUSB] AA bulk read partial: 0x%08x kept=%d submit=%u complete=%u\n",
                (unsigned int)result,
                count,
                g_bulkShared->readSubmitCount,
                g_bulkShared->readCompleteCount
            );
            completeBulkReadRequest(request, count);
            submitBulkRead();
            return;
        }

        /*
         Only a zero-byte abort can be dropped without losing stream bytes.
         Keep this path for the version handshake and for persistent errors;
         a persistent error still retires the session so the app can show the
         reconnect screen.
        */
        if (result == (int)SCE_USBD_ERROR_8024000A &&
            g_bulkReadTransientFailures < AA_BULK_READ_MAX_TRANSIENT &&
            sessionIsOpen()) {
            ++g_bulkReadTransientFailures;
            ++g_bulkShared->readSequenceFailures;
            request->state = BULK_READ_SKIPPED;
            wakeReader();
            submitBulkRead();
            return;
        }

        request->state = BULK_READ_FREE;
        setUsbFailure(VITA_AUTO_USB_FAILURE_BULK_READ, result, count);
        return;
    }

    g_bulkReadTransientFailures = 0;

    if (count > AA_BULK_READ_SLOT_SIZE) {
        request->state = BULK_READ_FREE;
        setUsbFailure(VITA_AUTO_USB_FAILURE_READ_TOO_LARGE, -1, count);
        return;
    }
    // VERSION_RESPONSE = control/0x03, length 8, message ID 2, status 0.
    if (g_usb.state == USB_AA_VERSION_SENT &&
        count >= AA_VERSION_RESPONSE_SIZE &&
        request->buffer[AA_VERSION_RESPONSE_CHANNEL] == 0x00 &&
        request->buffer[AA_VERSION_RESPONSE_FLAGS] == 0x03 &&
        request->buffer[AA_VERSION_RESPONSE_LENGTH_HIGH] == 0x00 &&
        request->buffer[AA_VERSION_RESPONSE_LENGTH_LOW] == 0x08 &&
        request->buffer[AA_VERSION_RESPONSE_MESSAGE_HIGH] == 0x00 &&
        request->buffer[AA_VERSION_RESPONSE_MESSAGE_LOW] == 0x02 &&
        request->buffer[AA_VERSION_RESPONSE_STATUS_HIGH] == 0x00 &&
        request->buffer[AA_VERSION_RESPONSE_STATUS_LOW] == 0x00)
    {
        vitaUsbLogInfo(
            "[VitaAutoUSB] Android Auto version accepted: %u.%u\n",
            (request->buffer[AA_VERSION_RESPONSE_MAJOR_HIGH] << 8) |
                request->buffer[AA_VERSION_RESPONSE_MAJOR_LOW],
            (request->buffer[AA_VERSION_RESPONSE_MINOR_HIGH] << 8) |
                request->buffer[AA_VERSION_RESPONSE_MINOR_LOW]
        );
        g_usb.state = USB_AA_VERSION_COMPLETE;
        /*
         The version response is consumed by the kernel. A single USB bulk
         transfer can contain additional AA frames after the 12-byte version
         response; preserve those bytes instead of discarding the whole
         transfer, otherwise the app can start TLS in the middle of a frame.
        */
        if (count > AA_VERSION_RESPONSE_SIZE) {
            const int remaining = count - AA_VERSION_RESPONSE_SIZE;
            for (int index = 0; index < remaining; ++index)
                request->buffer[index] =
                    request->buffer[AA_VERSION_RESPONSE_SIZE + index];
            completeBulkReadRequest(request, remaining);
        } else {
            completeBulkReadRequest(request, 0);
        }
        submitBulkRead();
        return;
    }

    if (g_usb.state == USB_AA_VERSION_SENT) {
        request->state = BULK_READ_FREE;
        ksceDebugPrintf("[VitaAutoUSB] Unexpected AA bulk response (%d bytes)\n", count);
        setUsbFailure(VITA_AUTO_USB_FAILURE_VERSION_RESPONSE, 0, count);
        return;
    }

    completeBulkReadRequest(request, count);

    if (g_usb.state == USB_AA_VERSION_COMPLETE)
        submitBulkRead();
}

/* Publish at most one request, in submission order. An out-of-order completion
 or a full ring stays in its DMA slot until a later scheduler pass. */
static int publishCompletedBulkReads(void)
{
    int requestIndex;
    BulkReadRequest *request = NULL;

    /*
     A transient read failure leaves a SKIPPED request in submission order.
     Consume it when it reaches the publish sequence so later completed reads
     are not blocked by a packet that will never arrive.
    */
    for (;;) {
        request = NULL;
        for (requestIndex = 0;
             requestIndex < AA_BULK_READ_PIPELINE_DEPTH;
             ++requestIndex)
        {
            BulkReadRequest *candidate = &g_bulkReadRequests[requestIndex];
            if (candidate->state != BULK_READ_FREE &&
                candidate->submitSequence == g_bulkReadPublishSequence)
            {
                request = candidate;
                break;
            }
        }

        if (request == NULL)
            return 0;

        if (request->state == BULK_READ_SKIPPED) {
            request->submitSequence = 0;
            __sync_synchronize();
            request->state = BULK_READ_FREE;
            ++g_bulkReadPublishSequence;
            return 1;
        }

        if (request->state == BULK_READ_COMPLETE)
            break;

        return 0;
    }

    __sync_synchronize();
    if (request->length > 0) {
        const int slot = g_bulkShared->readHead;
        const int nextHead = (slot + 1) % AA_BULK_READ_SLOT_COUNT;
        if (nextHead == g_bulkShared->readTail)
            return 0;

        for (int index = 0; index < request->length; ++index)
            g_bulkReadRing[slot][index] = request->buffer[index];
        g_bulkShared->readChecksum[slot] = checksumBytes(
            g_bulkReadRing[slot],
            request->length
        );
        g_bulkShared->readSequence[slot] = g_bulkShared->readNextSequence++;
        g_bulkShared->readCompletedAtUs[slot] = request->completedAtUs;
        g_bulkShared->readLength[slot] = request->length;
        __sync_synchronize();
        g_bulkShared->readHead = nextHead;
        ++g_bulkShared->readPublishedCount;
        if (g_readEvent >= 0)
            ksceKernelSetEventFlag(g_readEvent, USB_EVENT_RX);
        {
            const uint32_t depth = bulkReadRingDepth(nextHead, g_bulkShared->readTail);
            if (depth > g_bulkShared->readMaxDepth)
                g_bulkShared->readMaxDepth = depth;
        }
    }

    request->length = 0;
    request->submitSequence = 0;
    request->completedAtUs = 0;
    __sync_synchronize();
    request->state = BULK_READ_FREE;
    ++g_bulkReadPublishSequence;
    return 1;
}

static int countOutstandingBulkReads(void)
{
    int index;
    int count = 0;

    for (index = 0; index < AA_BULK_READ_PIPELINE_DEPTH; ++index) {
        if (g_bulkReadRequests[index].state != BULK_READ_FREE)
            ++count;
    }
    return count;
}

static BulkReadRequest *findFreeBulkReadRequest(void)
{
    int index;

    for (index = 0; index < AA_BULK_READ_PIPELINE_DEPTH; ++index) {
        if (g_bulkReadRequests[index].state == BULK_READ_FREE)
            return &g_bulkReadRequests[index];
    }
    return NULL;
}

/* Callbacks and the reader can both request a refill. Only the scheduler owner
 publishes packets or submits transfers; reentrant calls set Pending so the
 owner checks for more work before releasing the scheduler. */
static int submitBulkRead(void)
{
    int failed = 0;
    int runAgain;

    if (!sessionIsOpen())
        return -1;
    if (g_usb.bulkInPipe < 0)
        return 0;

    g_bulkReadSchedulerPending = 1;
    if (!__sync_bool_compare_and_swap(&g_bulkReadSchedulerBusy, 0, 1))
        return 0;

    do {
        int madeProgress;
        g_bulkReadSchedulerPending = 0;

        do {
            int outstanding;
            uint32_t reservedDepth;
            madeProgress = 0;

            while (publishCompletedBulkReads() > 0)
                madeProgress = 1;

            outstanding = countOutstandingBulkReads();
            /* Reserve ring space for every outstanding request. One slot stays
             empty so equal head/tail indices unambiguously mean an empty ring. */
            reservedDepth =
                bulkReadRingDepth(g_bulkShared->readHead, g_bulkShared->readTail) +
                (uint32_t)outstanding;

            while (sessionIsOpen() && !failed &&
                   !__sync_fetch_and_add(&g_bulkReadsDraining, 0) &&
                   outstanding < g_bulkReadConcurrencyLimit &&
                   reservedDepth < AA_BULK_READ_SLOT_COUNT - 1)
            {
                BulkReadRequest *request = findFreeBulkReadRequest();
                int submitResult;
                if (request == NULL)
                    break;

                request->submitSequence = g_bulkReadSubmitSequence++;
                request->length = 0;
                request->completedAtUs = 0;
                request->state = BULK_READ_SUBMITTING;
                __sync_add_and_fetch(&g_bulkShared->readArmed, 1);
                ++g_bulkShared->readSubmitCount;

                /*
                 Submit a full ring slot (4096 bytes), not one endpoint packet.
                 wMaxPacketSize is the hardware packet size, not a transfer
                 size: using it queued one 512-byte URB per packet, so the
                 32-slot ring only ever held 16 KB of stream and the
                 scheduler spent most of the session stalled on ring space
                 (observed depth 20-31 of 32). Each URB also costs a callback
                 and a reader wake, which is what pushed the app behind in the
                 first place.
                */
                submitResult = queueBulkTransfer(
                    g_usb.bulkInPipe,
                    request->buffer,
                    AA_BULK_READ_SLOT_SIZE,
                    aaBulkReadDone,
                    request
                );
                if (submitResult < 0) {
                    __sync_sub_and_fetch(&g_bulkShared->readArmed, 1);
                    --g_bulkShared->readSubmitCount;
                    --g_bulkReadSubmitSequence;
                    request->submitSequence = 0;
                    request->state = BULK_READ_FREE;

                    /* Some Vita USBD revisions may permit only one queued
                     transfer per pipe. Retain a working pipeline rather
                     than failing the AA session in that case. */
                    if (outstanding > 0) {
                        g_bulkReadConcurrencyLimit = outstanding;
                        ksceDebugPrintf(
                            "[VitaAutoUSB] RX pipeline limited to %d requests (0x%08x)\n",
                            outstanding,
                            submitResult
                        );
                        break;
                    }

                    failed = submitResult;
                    break;
                }

                /* A very fast completion may have changed SUBMITTING to
                 COMPLETE from inside ksceUsbdBulkTransfer. */
                __sync_bool_compare_and_swap(
                    &request->state,
                    BULK_READ_SUBMITTING,
                    BULK_READ_ACTIVE
                );
                ++outstanding;
                ++reservedDepth;
                madeProgress = 1;
            }
        } while (madeProgress && !failed);

        runAgain = __sync_lock_test_and_set(&g_bulkReadSchedulerPending, 0);
    } while (runAgain && !failed);

    g_bulkReadSchedulerBusy = 0;
    __sync_synchronize();
    if (g_bulkReadSchedulerPending && !failed)
        return submitBulkRead();

    if (failed < 0)
        setUsbFailure(VITA_AUTO_USB_FAILURE_BULK_READ_SUBMIT, failed, 0);
    return failed < 0 ? -1 : 0;
}

