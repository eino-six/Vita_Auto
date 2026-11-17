#include "video/screensaver.h"

#include "app/config.h"
#include "logger/vita_log.h"
#include "video/gxm_scaler.h"

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <psp2/kernel/threadmgr.h>
#include <psp2/power.h>
#include <psp2/rtc.h>
#include <vita2d.h>

namespace video {
namespace screensaver {
namespace {

// A one-pixel move every two minutes is invisible in motion but keeps static
// video edges from aging the same OLED pixels for the whole ride.
constexpr uint64_t PIXEL_SHIFT_INTERVAL_US = 2ull * 60ull * 1000000ull;
constexpr int8_t PIXEL_SHIFT_PATTERN[][2] = {
    { 0,  0},
    { 1,  0},
    { 2,  1},
    { 1,  2},
    { 0,  2},
    {-1,  2},
    {-2,  1},
    {-2,  0},
    {-1, -1},
    { 0, -2},
    { 1, -2},
    { 2, -1},
};

constexpr uint64_t CLOCK_CHECK_INTERVAL_US = 500000;

constexpr unsigned int CLOCK_COLOR = 0xffdededeu;
constexpr unsigned int LABEL_COLOR = 0xffb4b4b4u;
constexpr unsigned int CUTOUT_COLOR = 0xff000000u;

constexpr int SCREEN_WIDTH = app::config::SCREEN_WIDTH;
constexpr int SCREEN_HEIGHT = app::config::SCREEN_HEIGHT;

// Large centered seven-segment clock, matching the reference layout: day and
// battery above, date below, AM/PM to the right of the digits. Segments meet
// at 45-degree miters so lit neighbouring segments form one connected glyph
// instead of leaving notches between the horizontal and vertical bars.
constexpr int CLOCK_DIGIT_WIDTH = 104;
constexpr int CLOCK_DIGIT_HEIGHT = 200;
constexpr int CLOCK_SEGMENT_THICKNESS = 24;
constexpr int CLOCK_DIGIT_GAP = 14;
constexpr int CLOCK_COLON_WIDTH = 40;
constexpr int CLOCK_CORNER_CHAMFER = 12;

constexpr int CLOCK_WIDTH =
    4 * CLOCK_DIGIT_WIDTH + 4 * CLOCK_DIGIT_GAP + CLOCK_COLON_WIDTH;
constexpr int CLOCK_LEFT = (SCREEN_WIDTH - CLOCK_WIDTH) / 2;
constexpr int CLOCK_TOP = 168;
constexpr int CLOCK_BOTTOM = CLOCK_TOP + CLOCK_DIGIT_HEIGHT;
constexpr int LABEL_TOP = 112;
// Visually match the weekday-to-clock gap (~27 px on the default PVF) so the
// date sits as far below the digits as TUESDAY sits above them.
constexpr int DATE_TOP = CLOCK_BOTTOM + 37;
constexpr float LABEL_SCALE = 2.0f;
constexpr float AMPM_SCALE = 1.6f;
constexpr float SECONDS_SCALE = 1.6f;

std::atomic<bool> g_active{false};
std::atomic<bool> g_dirty{false};
std::atomic<bool> g_wakeRedraw{false};

vita2d_pvf* g_clockFont = nullptr;
bool g_loggedFontFailure = false;
uint64_t g_nextClockCheckUs = 0;
int g_lastClockMinute = -1;
int g_lastClockSecond = -1;
int g_lastBatteryPercent = -2;
uint64_t g_pixelShiftEpochUs = 0;

vita2d_pvf* clockFont()
{
    if (g_clockFont == nullptr) {
        g_clockFont = vita2d_load_default_pvf();
        if (g_clockFont == nullptr && !g_loggedFontFailure) {
            vitaLog("screensaver: default PVF font load failed");
            g_loggedFontFailure = true;
        }
    }
    return g_clockFont;
}

// vita2d_pvf_draw_text takes a baseline, not a top edge. These helpers take
// the visual top so enlarged labels cannot creep upward into the clock.
int textLineHeight(const char* text, float scale)
{
    vita2d_pvf* const font = clockFont();
    if (font == nullptr || text == nullptr)
        return 0;
    return vita2d_pvf_text_height(font, scale, text);
}

void drawCenteredTextBaseline(
    const char* text,
    int centerX,
    int baseline,
    float scale,
    unsigned int color
)
{
    vita2d_pvf* const font = clockFont();
    if (font == nullptr || text == nullptr)
        return;

    const int width = vita2d_pvf_text_width(font, scale, text);
    vita2d_pvf_draw_text(
        font, centerX - width / 2, baseline, color, scale, text);
}

void drawCenteredText(
    const char* text,
    int centerX,
    int top,
    float scale,
    unsigned int color
)
{
    drawCenteredTextBaseline(
        text,
        centerX,
        top + textLineHeight(text, scale),
        scale,
        color
    );
}

void drawRightAlignedText(
    const char* text,
    int right,
    int top,
    float scale,
    unsigned int color
)
{
    vita2d_pvf* const font = clockFont();
    if (font == nullptr || text == nullptr)
        return;

    const int width = vita2d_pvf_text_width(font, scale, text);
    const int baseline = top + vita2d_pvf_text_height(font, scale, text);
    vita2d_pvf_draw_text(font, right - width, baseline, color, scale, text);
}

void drawPolygon(const vita2d_color_vertex* vertices, size_t count)
{
    if (vertices == nullptr || count < 3)
        return;

    vita2d_color_vertex* const triangles =
        static_cast<vita2d_color_vertex*>(vita2d_pool_memalign(
            (count - 2) * 3 * sizeof(vita2d_color_vertex),
            sizeof(vita2d_color_vertex)
        ));
    if (triangles == nullptr)
        return;

    size_t written = 0;
    for (size_t i = 1; i + 1 < count; ++i) {
        triangles[written++] = vertices[0];
        triangles[written++] = vertices[i];
        triangles[written++] = vertices[i + 1];
    }
    vita2d_draw_array(SCE_GXM_PRIMITIVE_TRIANGLES, triangles, written);
}

vita2d_color_vertex vertex(float x, float y, unsigned int color)
{
    vita2d_color_vertex value{};
    value.x = x;
    value.y = y;
    value.z = 0.5f;
    value.color = color;
    return value;
}

void drawSolidQuad(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    float x3, float y3,
    unsigned int color
)
{
    const vita2d_color_vertex vertices[4] = {
        vertex(x0, y0, color),
        vertex(x1, y1, color),
        vertex(x2, y2, color),
        vertex(x3, y3, color),
    };
    drawPolygon(vertices, 4);
}

void drawSolidTriangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    unsigned int color
)
{
    const vita2d_color_vertex vertices[3] = {
        vertex(x0, y0, color),
        vertex(x1, y1, color),
        vertex(x2, y2, color),
    };
    drawPolygon(vertices, 3);
}

constexpr uint8_t SEG_A = 1u << 0;
constexpr uint8_t SEG_B = 1u << 1;
constexpr uint8_t SEG_C = 1u << 2;
constexpr uint8_t SEG_D = 1u << 3;
constexpr uint8_t SEG_E = 1u << 4;
constexpr uint8_t SEG_F = 1u << 5;
constexpr uint8_t SEG_G = 1u << 6;

constexpr uint8_t DIGIT_SEGMENTS[10] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
    SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
    SEG_B | SEG_C | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
};

void drawDigit(int digit, float x, float y)
{
    if (digit < 0 || digit > 9)
        return;

    const uint8_t mask = DIGIT_SEGMENTS[digit];
    const float width = static_cast<float>(CLOCK_DIGIT_WIDTH);
    const float height = static_cast<float>(CLOCK_DIGIT_HEIGHT);
    const float thickness = static_cast<float>(CLOCK_SEGMENT_THICKNESS);
    const float half = thickness * 0.5f;
    const float middle = y + height * 0.5f;
    const float right = x + width;
    const float bottom = y + height;

    // Horizontal bars are full width and end in 45-degree miters.
    if (mask & SEG_A) {
        drawSolidQuad(
            x, y,
            right, y,
            right - thickness, y + thickness,
            x + thickness, y + thickness,
            CLOCK_COLOR
        );
    }
    if (mask & SEG_D) {
        drawSolidQuad(
            x, bottom,
            right, bottom,
            right - thickness, bottom - thickness,
            x + thickness, bottom - thickness,
            CLOCK_COLOR
        );
    }
    if (mask & SEG_G) {
        const vita2d_color_vertex vertices[6] = {
            vertex(x + thickness, middle - half, CLOCK_COLOR),
            vertex(right - thickness, middle - half, CLOCK_COLOR),
            vertex(right - half, middle, CLOCK_COLOR),
            vertex(right - thickness, middle + half, CLOCK_COLOR),
            vertex(x + thickness, middle + half, CLOCK_COLOR),
            vertex(x + half, middle, CLOCK_COLOR),
        };
        drawPolygon(vertices, 6);
    }

    // Vertical bars use the matching miter on their outer ends, so the two
    // halves of the frame meet without a notch when both segments are lit.
    if (mask & SEG_F) {
        drawSolidQuad(
            x, y,
            x + thickness, y + thickness,
            x + thickness, middle,
            x, middle,
            CLOCK_COLOR
        );
    }
    if (mask & SEG_B) {
        drawSolidQuad(
            right, y,
            right, middle,
            right - thickness, middle,
            right - thickness, y + thickness,
            CLOCK_COLOR
        );
    }
    if (mask & SEG_E) {
        drawSolidQuad(
            x, middle,
            x + thickness, middle,
            x + thickness, bottom - thickness,
            x, bottom,
            CLOCK_COLOR
        );
    }
    if (mask & SEG_C) {
        drawSolidQuad(
            right, middle,
            right - thickness, middle,
            right - thickness, bottom - thickness,
            right, bottom,
            CLOCK_COLOR
        );
    }

    // Punch out the two display windows with the background colour. When a
    // segment around a window is off, the window simply merges with that dark
    // area, which is exactly how a real seven-segment display looks.
    vita2d_draw_rectangle(
        x + thickness,
        y + thickness,
        width - 2.0f * thickness,
        (middle - half) - (y + thickness),
        CUTOUT_COLOR
    );
    vita2d_draw_rectangle(
        x + thickness,
        middle + half,
        width - 2.0f * thickness,
        (bottom - thickness) - (middle + half),
        CUTOUT_COLOR
    );

    // Clip the four outer corners so the frame matches the reference's
    // octagonal outline instead of ending in square blocks.
    const float chamfer = static_cast<float>(CLOCK_CORNER_CHAMFER);
    drawSolidTriangle(x, y, x + chamfer, y, x, y + chamfer, CUTOUT_COLOR);
    drawSolidTriangle(
        right, y, right - chamfer, y, right, y + chamfer, CUTOUT_COLOR);
    drawSolidTriangle(
        x, bottom, x + chamfer, bottom, x, bottom - chamfer, CUTOUT_COLOR);
    drawSolidTriangle(
        right, bottom, right - chamfer, bottom, right, bottom - chamfer,
        CUTOUT_COLOR
    );
}

void drawColon(float x, float y)
{
    const float centerX = x + static_cast<float>(CLOCK_COLON_WIDTH) * 0.5f;
    constexpr float DOT_SIZE = 18.0f;
    const float topY = y + static_cast<float>(CLOCK_DIGIT_HEIGHT) * 0.30f;
    const float bottomY = y + static_cast<float>(CLOCK_DIGIT_HEIGHT) * 0.70f;
    vita2d_draw_rectangle(
        centerX - DOT_SIZE * 0.5f,
        topY - DOT_SIZE * 0.5f,
        DOT_SIZE,
        DOT_SIZE,
        CLOCK_COLOR
    );
    vita2d_draw_rectangle(
        centerX - DOT_SIZE * 0.5f,
        bottomY - DOT_SIZE * 0.5f,
        DOT_SIZE,
        DOT_SIZE,
        CLOCK_COLOR
    );
}

const char* const WEEKDAYS[] = {
    "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY",
    "THURSDAY", "FRIDAY", "SATURDAY",
};
const char* const MONTHS[] = {
    "JANUARY", "FEBRUARY", "MARCH", "APRIL", "MAY", "JUNE",
    "JULY", "AUGUST", "SEPTEMBER", "OCTOBER", "NOVEMBER", "DECEMBER",
};

int weekdayIndex(int year, int month, int day)
{
    // Sakamoto's algorithm. 0 = Sunday.
    static const int offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (month < 3)
        --year;
    const int index = (year + year / 4 - year / 100 + year / 400 +
                       offsets[month - 1] + day) % 7;
    return index < 0 ? index + 7 : index;
}

int batteryPercent()
{
    const int percent = scePowerGetBatteryLifePercent();
    if (percent < 0)
        return -1;
    return percent > 100 ? 100 : percent;
}

int leadingDigitInset(const char* timeText)
{
    if (timeText == nullptr || timeText[0] != '1')
        return 0;

    // A seven-segment 1 only lights the right-hand verticals. Its left half is
    // empty, so center the visible time rather than the fixed digit cells.
    return CLOCK_DIGIT_WIDTH - CLOCK_SEGMENT_THICKNESS;
}

void renderClock(uint64_t nowUs)
{
    const bool firstDraw = g_dirty.exchange(false, std::memory_order_acq_rel);
    if (firstDraw) {
        // The last visible frame may still be sampled by GXM. Let it finish
        // before the blank scene replaces it.
        vita2d_wait_rendering_done();
    }

    SceDateTime time{};
    const int result = sceRtcGetCurrentClockLocalTime(&time);
    const int minute = result >= 0
        ? static_cast<int>(time.hour) * 60 + static_cast<int>(time.minute)
        : (g_lastClockMinute >= 0 ? g_lastClockMinute : 0);
    const int second = result >= 0
        ? static_cast<int>(time.second)
        : (g_lastClockSecond >= 0 ? g_lastClockSecond : 0);
    const int battery = batteryPercent();

    if (!firstDraw &&
        minute == g_lastClockMinute &&
        second == g_lastClockSecond &&
        battery == g_lastBatteryPercent)
    {
        g_nextClockCheckUs = nowUs + CLOCK_CHECK_INTERVAL_US;
        return;
    }

    char timeText[8];
    char ampmText[4];
    char secondsText[4];
    if (result >= 0) {
        int hour = static_cast<int>(time.hour) % 12;
        if (hour == 0)
            hour = 12;
        std::snprintf(
            timeText,
            sizeof(timeText),
            "%02d:%02d",
            hour,
            static_cast<int>(time.minute)
        );
        std::snprintf(
            ampmText,
            sizeof(ampmText),
            "%s",
            time.hour < 12 ? "AM" : "PM"
        );
        std::snprintf(
            secondsText,
            sizeof(secondsText),
            "%02d",
            static_cast<int>(time.second)
        );
    } else {
        std::snprintf(timeText, sizeof(timeText), "--:--");
        std::snprintf(ampmText, sizeof(ampmText), "--");
        std::snprintf(secondsText, sizeof(secondsText), "--");
    }

    char dayText[16] = "--";
    char dateText[40] = "--";
    if (result >= 0 &&
        time.month >= 1 && time.month <= 12 &&
        time.day >= 1 && time.day <= 31) {
        std::snprintf(
            dayText,
            sizeof(dayText),
            "%s",
            WEEKDAYS[weekdayIndex(time.year, time.month, time.day)]
        );
        std::snprintf(
            dateText,
            sizeof(dateText),
            "%s %u %u",
            MONTHS[time.month - 1],
            static_cast<unsigned>(time.day),
            static_cast<unsigned>(time.year)
        );
    }

    char batteryText[8];
    if (battery >= 0) {
        std::snprintf(batteryText, sizeof(batteryText), "%d%%", battery);
    } else {
        std::snprintf(batteryText, sizeof(batteryText), "--%%");
    }

    int shiftX = 0;
    int shiftY = 0;
    pixelShift(shiftX, shiftY);
    const int clockOffsetX = -leadingDigitInset(timeText) / 2;
    const float originX = static_cast<float>(shiftX + clockOffsetX);
    const float originY = static_cast<float>(shiftY);
    const int indicatorCenterX =
        CLOCK_LEFT + CLOCK_WIDTH + 36 + shiftX + clockOffsetX;

    vita2d_start_drawing();
    vita2d_clear_screen();

    drawCenteredText(
        dayText,
        SCREEN_WIDTH / 2 + shiftX,
        LABEL_TOP + shiftY,
        LABEL_SCALE,
        LABEL_COLOR
    );
    drawRightAlignedText(
        batteryText,
        SCREEN_WIDTH - 32 + shiftX,
        LABEL_TOP + shiftY,
        AMPM_SCALE,
        LABEL_COLOR
    );

    const int colonX =
        CLOCK_LEFT + 2 * (CLOCK_DIGIT_WIDTH + CLOCK_DIGIT_GAP);
    drawDigit(
        timeText[0] - '0',
        originX + static_cast<float>(CLOCK_LEFT),
        originY + static_cast<float>(CLOCK_TOP)
    );
    drawDigit(
        timeText[1] - '0',
        originX + static_cast<float>(CLOCK_LEFT + CLOCK_DIGIT_WIDTH +
                                     CLOCK_DIGIT_GAP),
        originY + static_cast<float>(CLOCK_TOP)
    );
    drawColon(
        originX + static_cast<float>(colonX),
        originY + static_cast<float>(CLOCK_TOP)
    );
    drawDigit(
        timeText[3] - '0',
        originX + static_cast<float>(colonX + CLOCK_COLON_WIDTH +
                                     CLOCK_DIGIT_GAP),
        originY + static_cast<float>(CLOCK_TOP)
    );
    drawDigit(
        timeText[4] - '0',
        originX + static_cast<float>(colonX + CLOCK_COLON_WIDTH +
                                     2 * CLOCK_DIGIT_GAP + CLOCK_DIGIT_WIDTH),
        originY + static_cast<float>(CLOCK_TOP)
    );

    drawCenteredText(
        ampmText,
        indicatorCenterX,
        CLOCK_TOP + 2 + shiftY,
        AMPM_SCALE,
        LABEL_COLOR
    );
    drawCenteredTextBaseline(
        secondsText,
        indicatorCenterX,
        CLOCK_BOTTOM - 2 + shiftY,
        SECONDS_SCALE,
        LABEL_COLOR
    );
    drawCenteredText(
        dateText,
        SCREEN_WIDTH / 2 + shiftX,
        DATE_TOP + shiftY,
        LABEL_SCALE,
        LABEL_COLOR
    );

    vita2d_end_drawing();
    vita2d_swap_buffers();

    g_lastClockMinute = minute;
    g_lastClockSecond = second;
    g_lastBatteryPercent = battery;
    g_nextClockCheckUs = nowUs + CLOCK_CHECK_INTERVAL_US;
}

} // namespace

bool active()
{
    return g_active.load(std::memory_order_acquire);
}

bool activate()
{
    if (!video::isGxmScalerReady())
        return false;

    bool expected = false;
    if (!g_active.compare_exchange_strong(
            expected,
            true,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
    {
        return false;
    }

    g_dirty.store(true, std::memory_order_release);
    g_lastClockMinute = -1;
    g_lastClockSecond = -1;
    g_lastBatteryPercent = -2;
    g_nextClockCheckUs = 0;
    vitaLogInfo("screensaver: on");
    return true;
}

void wake()
{
    if (!g_active.exchange(false, std::memory_order_acq_rel))
        return;

    g_dirty.store(false, std::memory_order_release);
    g_wakeRedraw.store(true, std::memory_order_release);
    vitaLogInfo("screensaver: off (wake)");
}

void wakeForStatus()
{
    if (!g_active.exchange(false, std::memory_order_acq_rel))
        return;

    g_dirty.store(false, std::memory_order_release);
    g_wakeRedraw.store(false, std::memory_order_release);
    vitaLogInfo("screensaver: off (status)");
}

bool takeWakeRedraw()
{
    return g_wakeRedraw.exchange(false, std::memory_order_acq_rel);
}

void tick()
{
    if (!active())
        return;

    const uint64_t nowUs = sceKernelGetSystemTimeWide();
    if (!g_dirty.load(std::memory_order_acquire) &&
        nowUs < g_nextClockCheckUs)
    {
        return;
    }
    renderClock(nowUs);
}

void pixelShift(int& x, int& y)
{
    const uint64_t nowUs = sceKernelGetSystemTimeWide();
    if (g_pixelShiftEpochUs == 0)
        g_pixelShiftEpochUs = nowUs;

    const uint64_t step =
        (nowUs - g_pixelShiftEpochUs) / PIXEL_SHIFT_INTERVAL_US;
    const size_t index = static_cast<size_t>(step) %
        (sizeof(PIXEL_SHIFT_PATTERN) / sizeof(PIXEL_SHIFT_PATTERN[0]));
    x = PIXEL_SHIFT_PATTERN[index][0];
    y = PIXEL_SHIFT_PATTERN[index][1];
}

void shutdown()
{
    g_active.store(false, std::memory_order_release);
    g_dirty.store(false, std::memory_order_release);
    g_wakeRedraw.store(false, std::memory_order_release);

    if (g_clockFont != nullptr) {
        vita2d_wait_rendering_done();
        vita2d_free_pvf(g_clockFont);
        g_clockFont = nullptr;
    }
}

} // namespace screensaver
} // namespace video


