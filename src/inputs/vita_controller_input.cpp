#include "inputs/vita_controller_input.h"

#include "app/app_errors.h"
#include "logger/vita_log.h"

#include <cstddef>
#include <psp2/ctrl.h>
#include <psp2/kernel/threadmgr.h>

namespace inputs {
namespace {

static bool g_controllerSamplingStarted = false;
static SceCtrlPadInputMode g_previousControllerMode = SCE_CTRL_MODE_DIGITAL;
static uint32_t g_reportedControllerButtons = 0;
static uint32_t g_reportedFocusDirections = 0;
static uint64_t g_focusRepeatAtUs[4]{};

constexpr uint64_t NAVIGATION_REPEAT_DELAY_US = 350000;
constexpr uint64_t NAVIGATION_REPEAT_INTERVAL_US = 120000;

struct ButtonMapping {
    uint32_t mask;
    ButtonCommand command;
};

template <size_t Count>
void sendButtonChanges(
    const ButtonMapping (&mappings)[Count],
    uint32_t current,
    uint32_t& reported,
    ButtonSender sender
) {
    for (const ButtonMapping& mapping : mappings) {
        const bool pressed = (current & mapping.mask) != 0;
        if (pressed == ((reported & mapping.mask) != 0))
            continue;

        ButtonEvent event;
        event.command = mapping.command;
        event.pressed = pressed;
        if (!sender(event))
            continue;

        if (pressed)
            reported |= mapping.mask;
        else
            reported &= ~mapping.mask;
    }
}

template <size_t Count>
void sendNavigationChanges(
    const ButtonMapping (&mappings)[Count],
    uint32_t current,
    uint32_t& reported,
    uint64_t (&repeatAtUs)[Count],
    uint64_t nowUs,
    ButtonSender sender
)
{
    for (size_t index = 0; index < Count; ++index) {
        const ButtonMapping& mapping = mappings[index];
        const bool pressed = (current & mapping.mask) != 0;
        const bool wasPressed = (reported & mapping.mask) != 0;

        if (pressed != wasPressed) {
            ButtonEvent event;
            event.command = mapping.command;
            event.pressed = pressed;
            if (!sender(event))
                continue;

            if (pressed) {
                reported |= mapping.mask;
                repeatAtUs[index] = nowUs + NAVIGATION_REPEAT_DELAY_US;
            } else {
                reported &= ~mapping.mask;
                repeatAtUs[index] = 0;
            }
            continue;
        }

        if (!pressed || nowUs < repeatAtUs[index])
            continue;

        ButtonEvent event;
        event.command = mapping.command;
        event.pressed = true;
        event.repeated = true;
        if (sender(event)) {
            repeatAtUs[index] = nowUs + NAVIGATION_REPEAT_INTERVAL_US;
        }
    }
}

} // namespace

void initControllerInput()
{
    const int previousMode = sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
    if (previousMode < 0)
        vitaLogAppError(app::AppError::ControllerSamplingStartFailed, previousMode);
    g_controllerSamplingStarted = previousMode >= 0;
    if (g_controllerSamplingStarted) {
        g_previousControllerMode = static_cast<SceCtrlPadInputMode>(previousMode);
    }
    g_reportedControllerButtons = 0;
    g_reportedFocusDirections = 0;
    for (size_t index = 0; index < 4; ++index)
        g_focusRepeatAtUs[index] = 0;
    vitaLogInfo("controller init mode=0x%08x", previousMode);
}

void stopControllerInput()
{
    if (g_controllerSamplingStarted)
        sceCtrlSetSamplingMode(g_previousControllerMode);

    g_controllerSamplingStarted = false;
    g_reportedControllerButtons = 0;
    g_reportedFocusDirections = 0;
    for (size_t index = 0; index < 4; ++index)
        g_focusRepeatAtUs[index] = 0;
}

void pollControllerInput(ButtonSender sender)
{
    if (!g_controllerSamplingStarted || sender == nullptr)
        return;

    SceCtrlData pad{};
    const int sampleCount = sceCtrlPeekBufferPositive(0, &pad, 1);
    if (sampleCount <= 0)
        return;

    static const ButtonMapping buttonMappings[] = {
        { SCE_CTRL_CIRCLE,   inputs::ButtonCommand::Back },
        { SCE_CTRL_SQUARE,   inputs::ButtonCommand::VoiceAssistant },
        { SCE_CTRL_CROSS,    inputs::ButtonCommand::Select },
        { SCE_CTRL_LTRIGGER, inputs::ButtonCommand::Previous },
        { SCE_CTRL_RTRIGGER, inputs::ButtonCommand::Next },
        { SCE_CTRL_TRIANGLE, inputs::ButtonCommand::PlayPause },
    };

    sendButtonChanges(buttonMappings, pad.buttons, g_reportedControllerButtons, sender);

    constexpr uint8_t NAV_UP = 1u << 0;
    constexpr uint8_t NAV_DOWN = 1u << 1;
    constexpr uint8_t NAV_LEFT = 1u << 2;
    constexpr uint8_t NAV_RIGHT = 1u << 3;
    constexpr uint8_t ANALOG_LOW = 64;
    constexpr uint8_t ANALOG_HIGH = 192;

    // D-pad and both sticks drive standard Android focus navigation. All four
    // directions are D-pad key events so 2D layouts such as the Android Auto
    // launcher grid move vertically as well as horizontally. No rotary event
    // is advertised, because the phone's launcher otherwise treats navigation
    // as a one-dimensional rotary list.
    uint8_t focus = 0;
    if ((pad.buttons & SCE_CTRL_UP) || pad.ly <= ANALOG_LOW ||
        pad.ry <= ANALOG_LOW)
        focus |= NAV_UP;
    if ((pad.buttons & SCE_CTRL_DOWN) || pad.ly >= ANALOG_HIGH ||
        pad.ry >= ANALOG_HIGH)
        focus |= NAV_DOWN;
    if ((pad.buttons & SCE_CTRL_LEFT) || pad.lx <= ANALOG_LOW ||
        pad.rx <= ANALOG_LOW)
        focus |= NAV_LEFT;
    if ((pad.buttons & SCE_CTRL_RIGHT) || pad.lx >= ANALOG_HIGH ||
        pad.rx >= ANALOG_HIGH)
        focus |= NAV_RIGHT;

    static const ButtonMapping focusMappings[] = {
        { NAV_UP,    inputs::ButtonCommand::Up },
        { NAV_DOWN,  inputs::ButtonCommand::Down },
        { NAV_LEFT,  inputs::ButtonCommand::Left },
        { NAV_RIGHT, inputs::ButtonCommand::Right },
    };
    sendNavigationChanges(
        focusMappings,
        focus,
        g_reportedFocusDirections,
        g_focusRepeatAtUs,
        sceKernelGetSystemTimeWide(),
        sender
    );
}

} // namespace inputs
