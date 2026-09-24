#include "motion_engine_api.h"
#include "motion-engine.h"
#include <cmath>
#include <cstring>
#include <algorithm>

using namespace motionengine;

extern "C" {

void* me_create() {
    return new MotionEngineCore();
}

void me_destroy(void* handle) {
    if (handle) {
        delete static_cast<MotionEngineCore*>(handle);
    }
}

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
) {
    if (!handle || !frame) return;
    MotionEngineCore* engine = static_cast<MotionEngineCore*>(handle);

    GrayImage gray;
    MotionEngineCore::convertToGray(frame, width, height, static_cast<FrameFormat>(format), gray);
    engine->initBoundingBox(gray, BoundingBox(x, y, w, h));
}

int me_track_bounding_box(
    void* handle,
    const uint8_t* frame,
    int width,
    int height,
    int format,
    float outBox[5],
    float outMatrix[9]
) {
    if (!handle || !frame) return 0;
    MotionEngineCore* engine = static_cast<MotionEngineCore*>(handle);

    GrayImage gray;
    MotionEngineCore::convertToGray(frame, width, height, static_cast<FrameFormat>(format), gray);

    BoundingBox resultBox;
    bool success = engine->trackBoundingBox(gray, resultBox, outMatrix);

    outBox[0] = resultBox.x;
    outBox[1] = resultBox.y;
    outBox[2] = resultBox.width;
    outBox[3] = resultBox.height;
    outBox[4] = resultBox.confidence;

    return success ? 1 : 0;
}

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
) {
    if (!handle || !prevFrame || !currFrame || count <= 0) return 0;
    MotionEngineCore* engine = static_cast<MotionEngineCore*>(handle);

    GrayImage prevGray, currGray;
    MotionEngineCore::convertToGray(prevFrame, width, height, static_cast<FrameFormat>(format), prevGray);
    MotionEngineCore::convertToGray(currFrame, width, height, static_cast<FrameFormat>(format), currGray);

    std::vector<Point2f> ptsIn(count);
    for (int i = 0; i < count; ++i) {
        ptsIn[i] = Point2f(inPoints[i * 2], inPoints[i * 2 + 1], 1);
    }

    std::vector<Point2f> ptsOut;
    engine->trackPoints(prevGray, currGray, ptsIn, ptsOut, winSize, maxLevels, maxIters);

    int trackedCount = 0;
    for (int i = 0; i < count; ++i) {
        outPoints[i * 2] = ptsOut[i].x;
        outPoints[i * 2 + 1] = ptsOut[i].y;
        outStatus[i] = ptsOut[i].status;
        if (ptsOut[i].status) trackedCount++;
    }

    return trackedCount;
}

void me_stabilize_frame(
    void* handle,
    const uint8_t* frame,
    int width,
    int height,
    int format,
    float outData[15]
) {
    if (!handle || !frame) return;
    MotionEngineCore* engine = static_cast<MotionEngineCore*>(handle);

    GrayImage gray;
    MotionEngineCore::convertToGray(frame, width, height, static_cast<FrameFormat>(format), gray);

    StabilizationData stabData;
    engine->stabilizeFrame(gray, stabData);

    outData[0] = stabData.deltaX;
    outData[1] = stabData.deltaY;
    outData[2] = stabData.deltaAngle;
    outData[3] = stabData.smoothedX;
    outData[4] = stabData.smoothedY;
    outData[5] = stabData.smoothedAngle;
    std::memcpy(&outData[6], stabData.transformMatrix, 9 * sizeof(float));
}

void me_reset_stabilization(void* handle) {
    if (handle) {
        static_cast<MotionEngineCore*>(handle)->resetStabilization();
    }
}

static inline void sampleBilinearRGB(
    const uint8_t* src,
    int w,
    int h,
    float u,
    float v,
    uint8_t outRGB[3]
) {
    if (u < 0.0f || u >= w - 1.0f || v < 0.0f || v >= h - 1.0f) {
        // Clamp to nearest border pixel with subtle edge fade
        int cu = std::max(0, std::min(w - 1, static_cast<int>(std::round(u))));
        int cv = std::max(0, std::min(h - 1, static_cast<int>(std::round(v))));
        int idx = (cv * w + cu) * 3;
        outRGB[0] = src[idx] >> 1;     // Darken out-of-bounds border
        outRGB[1] = src[idx + 1] >> 1;
        outRGB[2] = src[idx + 2] >> 1;
        return;
    }

    int x0 = static_cast<int>(u);
    int y0 = static_cast<int>(v);
    float ax = u - static_cast<float>(x0);
    float ay = v - static_cast<float>(y0);

    int idx00 = (y0 * w + x0) * 3;
    int idx10 = idx00 + 3;
    int idx01 = ((y0 + 1) * w + x0) * 3;
    int idx11 = idx01 + 3;

    for (int c = 0; c < 3; ++c) {
        float p00 = src[idx00 + c];
        float p10 = src[idx10 + c];
        float p01 = src[idx01 + c];
        float p11 = src[idx11 + c];

        float val = (1.0f - ax) * (1.0f - ay) * p00 +
                    ax * (1.0f - ay) * p10 +
                    (1.0f - ax) * ay * p01 +
                    ax * ay * p11;

        outRGB[c] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, val)));
    }
}

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
) {
    if (!src || !dst || inW <= 0 || inH <= 0 || outW <= 0 || outH <= 0) return;
    float z = std::max(0.2f, zoom);
    float invZ = 1.0f / z;

    float halfOutW = outW * 0.5f;
    float halfOutH = outH * 0.5f;

    for (int oy = 0; oy < outH; ++oy) {
        float v = targetY + (oy - halfOutH) * invZ;
        uint8_t* dstRow = dst + (oy * outW) * 3;

        for (int ox = 0; ox < outW; ++ox) {
            float u = targetX + (ox - halfOutW) * invZ;
            sampleBilinearRGB(src, inW, inH, u, v, dstRow + ox * 3);
        }
    }

    if (drawReticle) {
        // Draw centered target box
        float scaledHalfBoxW = (boxW * 0.5f) * z;
        float scaledHalfBoxH = (boxH * 0.5f) * z;

        int minX = std::max(0, static_cast<int>(halfOutW - scaledHalfBoxW));
        int maxX = std::min(outW - 1, static_cast<int>(halfOutW + scaledHalfBoxW));
        int minY = std::max(0, static_cast<int>(halfOutH - scaledHalfBoxH));
        int maxY = std::min(outH - 1, static_cast<int>(halfOutH + scaledHalfBoxH));

        // Draw bounding box border (Cyan color: [0, 255, 255])
        auto drawPixel = [&](int px, int py, uint8_t r, uint8_t g, uint8_t b) {
            if (px >= 0 && px < outW && py >= 0 && py < outH) {
                int idx = (py * outW + px) * 3;
                dst[idx] = r;
                dst[idx + 1] = g;
                dst[idx + 2] = b;
            }
        };

        // Box corners
        int cornerLen = std::max(8, std::min(24, (maxX - minX) / 4));
        for (int x = minX; x <= minX + cornerLen; ++x) {
            drawPixel(x, minY, 0, 255, 255);
            drawPixel(x, minY + 1, 0, 255, 255);
            drawPixel(x, maxY, 0, 255, 255);
            drawPixel(x, maxY - 1, 0, 255, 255);
        }
        for (int x = maxX - cornerLen; x <= maxX; ++x) {
            drawPixel(x, minY, 0, 255, 255);
            drawPixel(x, minY + 1, 0, 255, 255);
            drawPixel(x, maxY, 0, 255, 255);
            drawPixel(x, maxY - 1, 0, 255, 255);
        }
        for (int y = minY; y <= minY + cornerLen; ++y) {
            drawPixel(minX, y, 0, 255, 255);
            drawPixel(minX + 1, y, 0, 255, 255);
            drawPixel(maxX, y, 0, 255, 255);
            drawPixel(maxX - 1, y, 0, 255, 255);
        }
        for (int y = maxY - cornerLen; y <= maxY; ++y) {
            drawPixel(minX, y, 0, 255, 255);
            drawPixel(minX + 1, y, 0, 255, 255);
            drawPixel(maxX, y, 0, 255, 255);
            drawPixel(maxX - 1, y, 0, 255, 255);
        }

        // Center crosshair (Red: [255, 50, 50])
        int cx = static_cast<int>(halfOutW);
        int cy = static_cast<int>(halfOutH);
        for (int d = -6; d <= 6; ++d) {
            if (d != 0) {
                drawPixel(cx + d, cy, 255, 60, 60);
                drawPixel(cx, cy + d, 255, 60, 60);
            }
        }
    }
}

} // extern "C"
