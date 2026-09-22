#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <deque>
#include <cmath>
#include <memory>
#include <algorithm>

namespace motionengine {

/**
 * Supported pixel format layouts for incoming Direct ByteBuffers.
 */
enum class FrameFormat : int32_t {
    RGBA = 0,
    RGB = 1,
    GRAYSCALE = 2,
    NV21 = 3
};

/**
 * 2D point representation with tracking status.
 */
struct Point2f {
    float x;
    float y;
    uint8_t status; // 1 = tracked successfully, 0 = lost/invalid

    Point2f() : x(0.0f), y(0.0f), status(0) {}
    Point2f(float _x, float _y, uint8_t _status = 1) : x(_x), y(_y), status(_status) {}
};

/**
 * Bounding box representation for object tracking.
 */
struct BoundingBox {
    float x;
    float y;
    float width;
    float height;
    float confidence;

    BoundingBox() : x(0.0f), y(0.0f), width(0.0f), height(0.0f), confidence(0.0f) {}
    BoundingBox(float _x, float _y, float _w, float _h, float _conf = 1.0f)
        : x(_x), y(_y), width(_w), height(_h), confidence(_conf) {}
};

/**
 * Video stabilization motion parameters and smoothed compensation matrix.
 */
struct StabilizationData {
    float deltaX;
    float deltaY;
    float deltaAngle;
    float smoothedX;
    float smoothedY;
    float smoothedAngle;
    float transformMatrix[9]; // 3x3 affine transformation matrix (Row-major)
};

/**
 * Rotoscoping mask contour containing polygon vertices and deformation matrix.
 */
struct MaskPolygon {
    std::vector<Point2f> vertices;
    float transformMatrix[9];
};

/**
 * High-performance 8-bit single-channel grayscale image buffer.
 */
struct GrayImage {
    int width;
    int height;
    std::vector<uint8_t> data;

    GrayImage() : width(0), height(0) {}
    GrayImage(int w, int h) : width(w), height(h), data(w * h, 0) {}

    inline uint8_t at(int x, int y) const {
        return data[y * width + x];
    }

    inline uint8_t& at(int x, int y) {
        return data[y * width + x];
    }

    inline const uint8_t* row(int y) const {
        return &data[y * width];
    }

    inline uint8_t* row(int y) {
        return &data[y * width];
    }
};

/**
 * Multi-scale image pyramid for pyramidal Lucas-Kanade optical flow.
 */
struct ImagePyramid {
    static constexpr int MAX_LEVELS = 3;
    std::vector<GrayImage> levels;

    void build(const GrayImage& base, int numLevels = MAX_LEVELS);
};

/**
 * Classical Computer Vision Core Motion Engine.
 */
class MotionEngineCore {
public:
    MotionEngineCore();
    ~MotionEngineCore() = default;

    // Buffer conversion: convert raw byte buffer to 8-bit grayscale
    static void convertToGray(
        const uint8_t* src,
        int width,
        int height,
        FrameFormat format,
        GrayImage& dst
    );

    // 1. Sparse Lucas-Kanade Optical Flow
    void trackPoints(
        const GrayImage& prevGray,
        const GrayImage& currGray,
        const std::vector<Point2f>& inPoints,
        std::vector<Point2f>& outPoints,
        int winSize = 15,
        int maxLevels = 3,
        int maxIters = 10,
        float minEigThreshold = 0.001f
    );

    // 2. Object Tracking: Template / Bounding-Box Matrix
    void initBoundingBox(
        const GrayImage& frame,
        const BoundingBox& box
    );

    bool trackBoundingBox(
        const GrayImage& frame,
        BoundingBox& outBox,
        float outMatrix[9]
    );

    // 3. Video Stabilization: Affine Motion Estimation + Moving Average Filter
    void stabilizeFrame(
        const GrayImage& frame,
        StabilizationData& outData
    );

    void resetStabilization();

    void setStabilizationSmoothingWindow(size_t windowSize) {
        mStabilizationWindowSize = std::max<size_t>(3, windowSize);
    }

    // 4. Rotoscoping/Masking: Contour/Vertex Deformation Tracking
    void trackPolygonMask(
        const GrayImage& prevGray,
        const GrayImage& currGray,
        const std::vector<Point2f>& inVertices,
        std::vector<Point2f>& outVertices,
        float outMatrix[9]
    );

private:
    // Lucas-Kanade optical flow on a single pyramid level
    void trackPointsSingleLevel(
        const GrayImage& imgI,
        const GrayImage& imgJ,
        const std::vector<Point2f>& ptsI,
        const std::vector<Point2f>& guessJ,
        std::vector<Point2f>& ptsJ,
        std::vector<uint8_t>& status,
        int winSize,
        int maxIters,
        float minEigThreshold
    );

    // Bilinear interpolation of subpixel intensity
    static inline float getSubpixel(const GrayImage& img, float x, float y);

    // Template tracking state
    bool mHasTemplate;
    BoundingBox mCurrentBox;
    GrayImage mTemplatePatch;
    float mTemplateMean;
    float mTemplateStdDev;

    // Video stabilization state
    bool mHasPrevStabFrame;
    GrayImage mPrevStabFrame;
    size_t mStabilizationWindowSize;

    struct TrajectoryPoint {
        float x;
        float y;
        float angle;
    };

    TrajectoryPoint mCurrentCumulativeTrajectory;
    std::deque<TrajectoryPoint> mTrajectoryHistory;

    // Estimate affine transform (translation + rotation + uniform scale) using least squares
    static bool estimateAffineSimilarity(
        const std::vector<Point2f>& src,
        const std::vector<Point2f>& dst,
        float& outDx,
        float& outDy,
        float& outAngle,
        float& outScale,
        float outMatrix[9]
    );

    // Laplacian contour smoothing for rotoscoping polygons
    static void smoothContour(
        const std::vector<Point2f>& inContour,
        std::vector<Point2f>& outContour,
        float alpha = 0.2f
    );
};

} // namespace motionengine

#endif // MOTION_ENGINE_H
