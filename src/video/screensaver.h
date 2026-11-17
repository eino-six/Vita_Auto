#pragma once

namespace video {
namespace screensaver {

// True while the display is showing the black clock screen instead of video.
bool active();

// Called from the input worker after the first video frame. Returns false when
// the display path is not ready or the screensaver is already active.
bool activate();

// Called from the input worker when any physical button wakes the display.
void wake();

// Called on the main thread when a connection/error screen replaces video.
void wakeForStatus();

// Main thread consumes this once so the pipeline redraws the latest decoded
// picture after a wake even though no new frame sequence arrived.
bool takeWakeRedraw();

// Draws the black screen and clock when active and due. Main thread only.
void tick();

// Current pixel-shift offset for the live video texture. Main thread only.
void pixelShift(int& x, int& y);

// Frees the clock font before the GXM/vita2d context shuts down.
void shutdown();

} // namespace screensaver
} // namespace video
