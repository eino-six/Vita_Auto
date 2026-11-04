#include "inputs/vita_input.h"
#include "logger/performance.h"
#include "android_auto/aa_direct.h"
#include "inputs/input_event_queue.h"
#include "inputs/vita_controller_input.h"
#include "inputs/vita_touch_input.h"
#include "logger/vita_log.h"
#include "transport/transport.h"

#include <psp2/kernel/threadmgr.h>

namespace inputs {
namespace performance = logger::performance;

namespace {

constexpr int DIRECT_INPUT_THREAD_PRIORITY = 0x10000100;
constexpr int DIRECT_INPUT_THREAD_STACK_SIZE = 0x10000;
constexpr size_t DIRECT_INPUT_QUEUE_CAPACITY = 64;
constexpr unsigned DIRECT_INPUT_POLL_US = 4000;

// A single input producer wakes the IO owner after queuing each event.
static inputs::InputEventQueue<DIRECT_INPUT_QUEUE_CAPACITY> g_directInputQueue;
static SceUID g_directInputThread = -1;
static volatile uint32_t g_directInputThreadRunning = 0;
static volatile uint32_t g_directInputReady = 0;
static DeliveryStats g_deliveryStats;
// v2 diagnostic: one sequence number per delivered key/touch edge/rotary step.
static uint32_t g_inputSequence = 0;

const char* touchActionName(inputs::TouchAction action)
{
    switch (action) {
    case inputs::TouchAction::Down: return "down";
    case inputs::TouchAction::Up:   return "up";
    case inputs::TouchAction::Move: return "move";
    default:                        return "unknown";
    }
}

android_auto::TouchAction toAndroidAutoTouchAction(inputs::TouchAction action)
{
    switch (action) {
    case inputs::TouchAction::Down:
        return android_auto::TouchAction::Down;
    case inputs::TouchAction::Up:
        return android_auto::TouchAction::Up;
    case inputs::TouchAction::Move:
    default:
        return android_auto::TouchAction::Move;
    }
}

uint32_t androidKeycode(inputs::ButtonCommand command)
{
    // android.view.KeyEvent constants advertised by service discovery.
    switch (command) {
    case inputs::ButtonCommand::Back:           return 4;
    case inputs::ButtonCommand::VoiceAssistant: return 84;
    case inputs::ButtonCommand::Select:         return 23; 
    case inputs::ButtonCommand::Up:             return 19;
    case inputs::ButtonCommand::Down:           return 20;
    case inputs::ButtonCommand::Left:           return 21;
    case inputs::ButtonCommand::Right:          return 22;
    case inputs::ButtonCommand::PlayPause:      return 85;
    case inputs::ButtonCommand::Next:           return 87;
    case inputs::ButtonCommand::Previous:       return 88;
    case inputs::ButtonCommand::Home:           return 3;
    default:                                    return 0;
    }
}

bool isRotaryCommand(inputs::ButtonCommand command)
{
    return command == inputs::ButtonCommand::PreviousItem ||
           command == inputs::ButtonCommand::NextItem;
}

bool sendControllerEvent(
    android_auto::DirectSession& session,
    const inputs::ButtonEvent& event
)
{
    if (isRotaryCommand(event.command))
    {
        return session.sendRotary(
            event.command == inputs::ButtonCommand::PreviousItem ? -1 : 1
        );
    }
    const uint32_t keycode = androidKeycode(event.command);
    if (keycode == 0)
        return false;
    // Repeat navigation with a new key edge, not another unmatched DOWN.
    if (event.repeated && !session.sendButton(keycode, false))
        return false;
    return session.sendButton(keycode, event.pressed);
}

bool queueTouchEvent(const inputs::TouchEvent& touch)
{
    inputs::InputEvent event;
    event.type = inputs::InputEventType::Touch;
    event.queuedAtUs = performance::nowUs();
    event.touch = touch;
    const bool queued = g_directInputQueue.push(event);
    if (queued)
        Transport::wake();
    return queued;
}

constexpr uint64_t SYNTHETIC_SWIPE_STEP_US = 12000;
constexpr uint64_t SYNTHETIC_SWIPE_REPEAT_GUARD_US = 200000;
constexpr uint16_t SYNTHETIC_SWIPE_X = 640;
constexpr uint16_t SYNTHETIC_SWIPE_TOP_Y = 260;
constexpr uint16_t SYNTHETIC_SWIPE_BOTTOM_Y = 460;
constexpr uint8_t SYNTHETIC_SWIPE_STEP_COUNT = 4;

struct SyntheticSwipeState {
    bool active = false;
    bool scrollUp = false;
    uint8_t step = 0;
    uint64_t nextStepAtUs = 0;
    uint64_t startedAtUs = 0;
};

static SyntheticSwipeState g_syntheticSwipe;

bool isScrollCommand(inputs::ButtonCommand command)
{
    return command == inputs::ButtonCommand::ScrollUp ||
           command == inputs::ButtonCommand::ScrollDown;
}

void startSyntheticSwipe(inputs::ButtonCommand command, bool repeated)
{
    if (g_syntheticSwipe.active)
        return;

    const uint64_t nowUs = performance::nowUs();
    if (repeated &&
        nowUs - g_syntheticSwipe.startedAtUs < SYNTHETIC_SWIPE_REPEAT_GUARD_US)
        return;

    g_syntheticSwipe.active = true;
    g_syntheticSwipe.scrollUp = command == inputs::ButtonCommand::ScrollUp;
    g_syntheticSwipe.step = 0;
    g_syntheticSwipe.nextStepAtUs = nowUs;
    g_syntheticSwipe.startedAtUs = nowUs;
    vitaLog(
        "direct-usb: synthetic swipe direction=%s",
        g_syntheticSwipe.scrollUp ? "up" : "down"
    );
}

void advanceSyntheticSwipe()
{
    if (!g_syntheticSwipe.active)
        return;

    const uint64_t nowUs = performance::nowUs();
    if (nowUs < g_syntheticSwipe.nextStepAtUs)
        return;

    const uint16_t startY = g_syntheticSwipe.scrollUp
        ? SYNTHETIC_SWIPE_TOP_Y
        : SYNTHETIC_SWIPE_BOTTOM_Y;
    const uint16_t endY = g_syntheticSwipe.scrollUp
        ? SYNTHETIC_SWIPE_BOTTOM_Y
        : SYNTHETIC_SWIPE_TOP_Y;

    inputs::TouchEvent touch;
    touch.x = SYNTHETIC_SWIPE_X;
    switch (g_syntheticSwipe.step) {
    case 0:
        touch.action = inputs::TouchAction::Down;
        touch.y = startY;
        break;
    case 1:
        touch.action = inputs::TouchAction::Move;
        touch.y = static_cast<uint16_t>((startY + endY) / 2);
        break;
    case 2:
        touch.action = inputs::TouchAction::Move;
        touch.y = endY;
        break;
    default:
        touch.action = inputs::TouchAction::Up;
        touch.y = endY;
        break;
    }

    if (!queueTouchEvent(touch))
        return;

    ++g_syntheticSwipe.step;
    if (g_syntheticSwipe.step >= SYNTHETIC_SWIPE_STEP_COUNT) {
        g_syntheticSwipe.active = false;
        return;
    }
    g_syntheticSwipe.nextStepAtUs = nowUs + SYNTHETIC_SWIPE_STEP_US;
}

bool queueButtonEvent(const inputs::ButtonEvent& button)
{
    if (isScrollCommand(button.command)) {
        if (button.pressed)
            startSyntheticSwipe(button.command, button.repeated);
        return true;
    }

    // Rotary releases only update the controller's local held-button state.
    if (isRotaryCommand(button.command) && !button.pressed)
        return true;
    inputs::InputEvent event;
    event.type = inputs::InputEventType::Button;
    event.queuedAtUs = performance::nowUs();
    event.button = button;
    const bool queued = g_directInputQueue.push(event);
    if (queued)
        Transport::wake();
    return queued;
}

bool sampleDirectInput()
{
    const bool inputReady = __atomic_load_n(
        &g_directInputReady,
        __ATOMIC_ACQUIRE
    ) != 0;

    if (!inputReady)
        return false;

    pollTouchInput(queueTouchEvent);
    pollControllerInput(queueButtonEvent);
    advanceSyntheticSwipe();
    return true;
}

int directInputThreadEntry(SceSize, void*)
{
    while (__atomic_load_n(
               &g_directInputThreadRunning,
               __ATOMIC_ACQUIRE) != 0)
    {
        sampleDirectInput();
        sceKernelDelayThread(DIRECT_INPUT_POLL_US);
    }
    return sceKernelExitThread(0);
}

bool startDirectInputThread()
{
    g_directInputThread = sceKernelCreateThread(
        "vita-auto-direct-input",
        directInputThreadEntry,
        DIRECT_INPUT_THREAD_PRIORITY,
        DIRECT_INPUT_THREAD_STACK_SIZE,
        0,
        0,
        nullptr
    );
    if (g_directInputThread < 0) {
        vitaLog(
            "[%s] direct-usb: input thread create failed err=0x%08x; main fallback", app::appErrorName(app::AppError::InputThreadCreateFailed),
            g_directInputThread
        );
        g_directInputThread = -1;
        return false;
    }

    __atomic_store_n(&g_directInputThreadRunning, 1u, __ATOMIC_RELEASE);
    const int result = sceKernelStartThread(g_directInputThread, 0, nullptr);
    if (result < 0) {
        vitaLog(
            "[%s] direct-usb: input thread start failed err=0x%08x; main fallback", app::appErrorName(app::AppError::InputThreadStartFailed),
            result
        );
        __atomic_store_n(&g_directInputThreadRunning, 0u, __ATOMIC_RELEASE);
        sceKernelDeleteThread(g_directInputThread);
        g_directInputThread = -1;
        return false;
    }

    vitaLogInfo("direct-usb: input thread started");
    return true;
}

void stopDirectInputThread()
{
    if (g_directInputThread < 0)
        return;

    __atomic_store_n(&g_directInputThreadRunning, 0u, __ATOMIC_RELEASE);
    sceKernelWaitThreadEnd(g_directInputThread, nullptr, nullptr);
    sceKernelDeleteThread(g_directInputThread);
    g_directInputThread = -1;
}

bool deliverDirectInputEvent(
    android_auto::DirectSession& session,
    const inputs::InputEvent& event
)
{
    uint32_t queueDelayUs = 0;
    if (event.queuedAtUs != 0) {
        const uint64_t nowUs = performance::nowUs();
        const uint64_t delayUs = nowUs > event.queuedAtUs
            ? nowUs - event.queuedAtUs
            : 0;
        const uint32_t boundedDelay = delayUs > UINT32_MAX
            ? UINT32_MAX
            : static_cast<uint32_t>(delayUs);
        queueDelayUs = boundedDelay;
        performance::record(performance::Metric::InputQueue, boundedDelay);
        if (boundedDelay > g_deliveryStats.maxQueueUs)
            g_deliveryStats.maxQueueUs = boundedDelay;
    }

    const bool inputReady = session.inputReady();
    const uint64_t encodeStartUs = performance::nowUs();
    bool sent = false;
    if (event.type == inputs::InputEventType::Touch) {
        sent = session.sendTouch(
            toAndroidAutoTouchAction(event.touch.action),
            event.touch.x,
            event.touch.y,
            event.touch.pointerId
        );
    } else {
        sent = sendControllerEvent(session, event.button);
    }
    performance::record(performance::Metric::InputEncode, performance::elapsedUs(encodeStartUs));
    const bool transmitted = inputReady && sent;

    // Touch moves are coalesced and can be very frequent. Log the edges that
    // produce an Android Auto action, plus every key/rotary report. A line is
    // emitted even when the send fails or the input channel is not open yet,
    // so a failed report is visible next to the USB detach.
    if (event.type == inputs::InputEventType::Touch) {
        if (event.touch.action != inputs::TouchAction::Move) {
            ++g_inputSequence;
            vitaLog(
                "direct-usb: input seq=%u type=touch action=%s x=%u y=%u ready=%u tx=%u queue_us=%u",
                g_inputSequence,
                touchActionName(event.touch.action),
                event.touch.x,
                event.touch.y,
                inputReady ? 1u : 0u,
                transmitted ? 1u : 0u,
                queueDelayUs
            );
        }
    } else {
        ++g_inputSequence;
        if (isRotaryCommand(event.button.command)) {
            vitaLog(
                "direct-usb: input seq=%u type=rotary delta=%d ready=%u tx=%u queue_us=%u",
                g_inputSequence,
                event.button.command == inputs::ButtonCommand::PreviousItem ? -1 : 1,
                inputReady ? 1u : 0u,
                transmitted ? 1u : 0u,
                queueDelayUs
            );
        } else {
            vitaLog(
                "direct-usb: input seq=%u type=button key=%u down=%u rep=%u ready=%u tx=%u queue_us=%u",
                g_inputSequence,
                androidKeycode(event.button.command),
                event.button.pressed ? 1u : 0u,
                event.button.repeated ? 1u : 0u,
                inputReady ? 1u : 0u,
                transmitted ? 1u : 0u,
                queueDelayUs
            );
        }
    }

    if (sent) {
        ++g_deliveryStats.delivered;
        if (event.type == inputs::InputEventType::Touch)
            ++g_deliveryStats.touchDelivered;
        else if (isRotaryCommand(event.button.command))
            ++g_deliveryStats.rotarySteps;
        else
            ++g_deliveryStats.buttonDelivered;
    }
    return sent;
}

bool drainDirectInputQueue(android_auto::DirectSession& session)
{
    inputs::InputEvent event;
    inputs::InputEvent latestMove;
    bool hasMove = false;

    while (g_directInputQueue.pop(event)) {
        ++g_deliveryStats.dequeued;
        const bool isMove =
            event.type == inputs::InputEventType::Touch &&
            event.touch.action == inputs::TouchAction::Move;
        if (isMove) {
            if (hasMove)
                ++g_deliveryStats.movesCoalesced;
            latestMove = event;
            hasMove = true;
            continue;
        }

        // Preserve event ordering across DOWN, UP, and every button edge, but
        // never make Android process obsolete intermediate drag positions.
        if (hasMove) {
            if (!deliverDirectInputEvent(session, latestMove))
                return false;
            hasMove = false;
        }
        if (!deliverDirectInputEvent(session, event))
            return false;
    }

    return !hasMove || deliverDirectInputEvent(session, latestMove);
}

} // namespace

void init()
{
    initTouchInput();
    initControllerInput();
    g_directInputQueue.reset();
    g_deliveryStats = {};
    g_inputSequence = 0;
    g_syntheticSwipe = {};
    setReady(false);
}

void stop()
{
    stopTouchInput();
    stopControllerInput();
}

bool startWorker() { return startDirectInputThread(); }
void stopWorker() { stopDirectInputThread(); }
bool sample() { return sampleDirectInput(); }

void setReady(bool ready)
{
    __atomic_store_n(&g_directInputReady, ready ? 1u : 0u, __ATOMIC_RELEASE);
}

bool drain(android_auto::DirectSession& session)
{
    return drainDirectInputQueue(session);
}

DeliveryStats deliveryStats() { return g_deliveryStats; }

} // namespace inputs
