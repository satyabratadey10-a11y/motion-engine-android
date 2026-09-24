#ifndef MOTION_ENGINE_API_H
#define MOTION_ENGINE_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Lifecycle
void* me_create();
void me_destroy(void* handle);

// Object Bounding Box Tracking
void me_init_bounding_box(
    void* handle,
    const uint8_t* frame,
    int width,
    int height,
    int format,
    float x,
    float y,
    float w,
    float h
);

int me_track_bounding_box(
    void* handle,
    const uint8_t* frame,
    int width,
    int height,
    int format,
    float outBox[5], // x, y, w, h, confidence
    float outMatrix[9] // 3x3 affine matrix
);

// Sparse Optical Flow (Lucas-Kanade)
int me_track_points(
    void* handle,
    const uint8_t* prevFrame,
    const uint8_t* currFrame,
    int width,
    int height,
    int format,
    const float* inPoints,
    float* outPoints,
    uint8_t* outStatus,
    int count,
    int winSize,
    int maxLevels,
    int maxIters
);

// Video Stabilization
void me_stabilize_frame(
    void* handle,
    const uint8_t* frame,
    int width,
    int height,
    int format,
    float outData[15]
);

void me_reset_stabilization(void* handle);

// Object Centering Warp Engine:
// Pins (targetX, targetY) at the exact center (outW/2, outH/2) of output canvas.
void me_center_frame_rgb(
    const uint8_t* src,
    int inW,
    int inH,
    uint8_t* dst,
    int outW,
    int outH,
    float targetX,
    float targetY,
    float zoom,
    int drawReticle,
    float boxW,
    float boxH
);

#ifdef __cplusplus
}
#endif

#endif // MOTION_ENGINE_API_H
