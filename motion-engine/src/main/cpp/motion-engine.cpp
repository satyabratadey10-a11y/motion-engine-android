#include "motion-engine.h"
#include <jni.h>
#include <android/log.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <vector>

#define LOG_TAG "MotionEngineNative"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace motionengine {

// ============================================================================
// Image Pyramid Construction
// ============================================================================

void ImagePyramid::build(const GrayImage& base, int numLevels) {
    int targetLevels = std::max(1, std::min(numLevels, MAX_LEVELS));
    levels.clear();
    levels.reserve(targetLevels);
    levels.push_back(base);

    for (int l = 1; l < targetLevels; ++l) {
        const GrayImage& prev = levels[l - 1];
        int newW = prev.width / 2;
        int newH = prev.height / 2;
        if (newW < 8 || newH < 8) {
            break;
        }

        GrayImage down(newW, newH);
        for (int y = 0; y < newH; ++y) {
            int y0 = y * 2;
            int y1 = std::min(y0 + 1, prev.height - 1);
            const uint8_t* r0 = prev.row(y0);
            const uint8_t* r1 = prev.row(y1);
            uint8_t* dstRow = &down.at(0, y);

            for (int x = 0; x < newW; ++x) {
                int x0 = x * 2;
                int x1 = std::min(x0 + 1, prev.width - 1);
                // 2x2 box filter with round-half-up
                uint32_t sum = static_cast<uint32_t>(r0[x0]) +
                               static_cast<uint32_t>(r0[x1]) +
                               static_cast<uint32_t>(r1[x0]) +
                               static_cast<uint32_t>(r1[x1]);
                dstRow[x] = static_cast<uint8_t>((sum + 2) >> 2);
            }
        }
        levels.push_back(std::move(down));
    }
}

// ============================================================================
// Core Motion Engine Implementation
// ============================================================================

MotionEngineCore::MotionEngineCore()
    : mHasTemplate(false),
      mTemplateMean(0.0f),
      mTemplateStdDev(1.0f),
      mHasPrevStabFrame(false),
      mStabilizationWindowSize(20) {
    mCurrentCumulativeTrajectory = {0.0f, 0.0f, 0.0f};
}

void MotionEngineCore::convertToGray(
    const uint8_t* src,
    int width,
    int height,
    FrameFormat format,
    GrayImage& dst
) {
    dst.width = width;
    dst.height = height;
    dst.data.resize(width * height);

    if (format == FrameFormat::GRAYSCALE || format == FrameFormat::NV21) {
        // NV21 Y-plane and raw Grayscale start with width * height 8-bit luminance bytes
        std::memcpy(dst.data.data(), src, width * height);
    } else if (format == FrameFormat::RGBA) {
        // Fast integer fixed-point RGB to Luma: (77*R + 150*G + 29*B) >> 8
        const uint8_t* pSrc = src;
        uint8_t* pDst = dst.data.data();
        int totalPixels = width * height;
        for (int i = 0; i < totalPixels; ++i) {
            uint32_t r = pSrc[0];
            uint32_t g = pSrc[1];
            uint32_t b = pSrc[2];
            *pDst++ = static_cast<uint8_t>((r * 77 + g * 150 + b * 29) >> 8);
            pSrc += 4;
        }
    } else if (format == FrameFormat::RGB) {
        const uint8_t* pSrc = src;
        uint8_t* pDst = dst.data.data();
        int totalPixels = width * height;
        for (int i = 0; i < totalPixels; ++i) {
            uint32_t r = pSrc[0];
            uint32_t g = pSrc[1];
            uint32_t b = pSrc[2];
            *pDst++ = static_cast<uint8_t>((r * 77 + g * 150 + b * 29) >> 8);
            pSrc += 3;
        }
    }
}

inline float MotionEngineCore::getSubpixel(const GrayImage& img, float x, float y) {
    if (x < 0.0f || x >= img.width - 1 || y < 0.0f || y >= img.height - 1) {
        int cx = std::max(0, std::min(img.width - 1, static_cast<int>(std::round(x))));
        int cy = std::max(0, std::min(img.height - 1, static_cast<int>(std::round(y))));
        return static_cast<float>(img.at(cx, cy));
    }

    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    float ax = x - static_cast<float>(x0);
    float ay = y - static_cast<float>(y0);

    const uint8_t* r0 = img.row(y0);
    const uint8_t* r1 = img.row(y0 + 1);

    float p00 = static_cast<float>(r0[x0]);
    float p10 = static_cast<float>(r0[x0 + 1]);
    float p01 = static_cast<float>(r1[x0]);
    float p11 = static_cast<float>(r1[x0 + 1]);

    return (1.0f - ax) * (1.0f - ay) * p00 +
           ax * (1.0f - ay) * p10 +
           (1.0f - ax) * ay * p01 +
           ax * ay * p11;
}

// ----------------------------------------------------------------------------
// 1. Sparse Lucas-Kanade Optical Flow
// ----------------------------------------------------------------------------

void MotionEngineCore::trackPointsSingleLevel(
    const GrayImage& imgI,
    const GrayImage& imgJ,
    const std::vector<Point2f>& ptsI,
    const std::vector<Point2f>& guessJ,
    std::vector<Point2f>& ptsJ,
    std::vector<uint8_t>& status,
    int winSize,
    int maxIters,
    float minEigThreshold
) {
    int halfWin = winSize / 2;
    size_t numPts = ptsI.size();

    for (size_t ptIdx = 0; ptIdx < numPts; ++ptIdx) {
        if (!status[ptIdx]) {
            continue;
        }

        float x0 = ptsI[ptIdx].x;
        float y0 = ptsI[ptIdx].y;

        // Check if window is safely inside image bounds
        if (x0 - halfWin < 1.0f || x0 + halfWin >= imgI.width - 2.0f ||
            y0 - halfWin < 1.0f || y0 + halfWin >= imgI.height - 2.0f) {
            status[ptIdx] = 0;
            continue;
        }

        // Precompute spatial gradients and structural tensor G on window in imgI
        float gxx = 0.0f;
        float gyy = 0.0f;
        float gxy = 0.0f;

        // Cache spatial gradient values
        int winArea = winSize * winSize;
        std::vector<float> gradX(winArea);
        std::vector<float> gradY(winArea);
        std::vector<float> patchI(winArea);

        int sampleIdx = 0;
        for (int wy = -halfWin; wy <= halfWin; ++wy) {
            for (int wx = -halfWin; wx <= halfWin; ++wx) {
                float px = x0 + static_cast<float>(wx);
                float py = y0 + static_cast<float>(wy);

                float val = getSubpixel(imgI, px, py);
                float dx = (getSubpixel(imgI, px + 1.0f, py) - getSubpixel(imgI, px - 1.0f, py)) * 0.5f;
                float dy = (getSubpixel(imgI, px, py + 1.0f) - getSubpixel(imgI, px, py - 1.0f)) * 0.5f;

                patchI[sampleIdx] = val;
                gradX[sampleIdx] = dx;
                gradY[sampleIdx] = dy;

                gxx += dx * dx;
                gyy += dy * dy;
                gxy += dx * dy;

                sampleIdx++;
            }
        }

        // Structural tensor condition check: minimum eigenvalue
        float trace = gxx + gyy;
        float det = gxx * gyy - gxy * gxy;
        float disc = std::sqrt(std::max(0.0f, (gxx - gyy) * (gxx - gyy) + 4.0f * gxy * gxy));
        float minEigenvalue = (trace - disc) * 0.5f;

        if (minEigenvalue < minEigThreshold || det <= 1e-6f) {
            status[ptIdx] = 0;
            continue;
        }

        float invDet = 1.0f / det;

        // Iterative Lucas-Kanade optimization
        float vx = guessJ[ptIdx].x - x0;
        float vy = guessJ[ptIdx].y - y0;

        bool converged = false;
        for (int iter = 0; iter < maxIters; ++iter) {
            float bx = 0.0f;
            float by = 0.0f;
            sampleIdx = 0;

            float curX = x0 + vx;
            float curY = y0 + vy;

            if (curX - halfWin < 0.0f || curX + halfWin >= imgJ.width - 1.0f ||
                curY - halfWin < 0.0f || curY + halfWin >= imgJ.height - 1.0f) {
                break;
            }

            for (int wy = -halfWin; wy <= halfWin; ++wy) {
                for (int wx = -halfWin; wx <= halfWin; ++wx) {
                    float jx = curX + static_cast<float>(wx);
                    float jy = curY + static_cast<float>(wy);

                    float valJ = getSubpixel(imgJ, jx, jy);
                    float diff = patchI[sampleIdx] - valJ;

                    bx += diff * gradX[sampleIdx];
                    by += diff * gradY[sampleIdx];

                    sampleIdx++;
                }
            }

            float etaX = (gyy * bx - gxy * by) * invDet;
            float etaY = (gxx * by - gxy * bx) * invDet;

            vx += etaX;
            vy += etaY;

            if (etaX * etaX + etaY * etaY < 0.0004f) { // Convergence threshold 0.02 pixel
                converged = true;
                break;
            }
        }

        float finalX = x0 + vx;
        float finalY = y0 + vy;

        if (finalX >= 0.0f && finalX < imgJ.width &&
            finalY >= 0.0f && finalY < imgJ.height) {
            ptsJ[ptIdx].x = finalX;
            ptsJ[ptIdx].y = finalY;
            ptsJ[ptIdx].status = 1;
        } else {
            status[ptIdx] = 0;
            ptsJ[ptIdx].status = 0;
        }
    }
}

void MotionEngineCore::trackPoints(
    const GrayImage& prevGray,
    const GrayImage& currGray,
    const std::vector<Point2f>& inPoints,
    std::vector<Point2f>& outPoints,
    int winSize,
    int maxLevels,
    int maxIters,
    float minEigThreshold
) {
    size_t numPts = inPoints.size();
    outPoints.resize(numPts);

    if (numPts == 0) return;

    ImagePyramid pyrPrev;
    ImagePyramid pyrCurr;
    pyrPrev.build(prevGray, maxLevels);
    pyrCurr.build(currGray, maxLevels);

    int actualLevels = static_cast<int>(std::min(pyrPrev.levels.size(), pyrCurr.levels.size()));

    std::vector<uint8_t> status(numPts, 1);
    std::vector<Point2f> currentGuess(numPts);

    for (size_t i = 0; i < numPts; ++i) {
        currentGuess[i] = inPoints[i];
        status[i] = inPoints[i].status;
    }

    // Coarse-to-fine propagation through pyramid levels
    for (int lvl = actualLevels - 1; lvl >= 0; --lvl) {
        float scale = 1.0f / static_cast<float>(1 << lvl);

        std::vector<Point2f> ptsLvlI(numPts);
        std::vector<Point2f> guessLvlJ(numPts);
        std::vector<Point2f> ptsLvlJ(numPts);

        for (size_t i = 0; i < numPts; ++i) {
            ptsLvlI[i] = Point2f(inPoints[i].x * scale, inPoints[i].y * scale, status[i]);
            guessLvlJ[i] = Point2f(currentGuess[i].x * scale, currentGuess[i].y * scale, status[i]);
            ptsLvlJ[i] = guessLvlJ[i];
        }

        trackPointsSingleLevel(
            pyrPrev.levels[lvl],
            pyrCurr.levels[lvl],
            ptsLvlI,
            guessLvlJ,
            ptsLvlJ,
            status,
            winSize,
            maxIters,
            minEigThreshold
        );

        float invScale = static_cast<float>(1 << lvl);
        for (size_t i = 0; i < numPts; ++i) {
            if (status[i]) {
                currentGuess[i].x = ptsLvlJ[i].x * invScale;
                currentGuess[i].y = ptsLvlJ[i].y * invScale;
            }
        }
    }

    for (size_t i = 0; i < numPts; ++i) {
        outPoints[i].x = currentGuess[i].x;
        outPoints[i].y = currentGuess[i].y;
        outPoints[i].status = status[i];
    }
}

// ----------------------------------------------------------------------------
// 2. Object Tracking: Template / Bounding-Box Tracking Matrix
// ----------------------------------------------------------------------------

void MotionEngineCore::initBoundingBox(const GrayImage& frame, const BoundingBox& box) {
    mCurrentBox = box;
    int bx = std::max(0, static_cast<int>(box.x));
    int by = std::max(0, static_cast<int>(box.y));
    int bw = std::max(4, std::min(static_cast<int>(box.width), frame.width - bx));
    int bh = std::max(4, std::min(static_cast<int>(box.height), frame.height - by));

    mTemplatePatch = GrayImage(bw, bh);
    float sum = 0.0f;
    float sumSq = 0.0f;
    int count = bw * bh;

    for (int y = 0; y < bh; ++y) {
        const uint8_t* srcRow = frame.row(by + y) + bx;
        uint8_t* dstRow = &mTemplatePatch.at(0, y);
        std::memcpy(dstRow, srcRow, bw);

        for (int x = 0; x < bw; ++x) {
            float v = static_cast<float>(dstRow[x]);
            sum += v;
            sumSq += v * v;
        }
    }

    mTemplateMean = sum / count;
    float variance = (sumSq / count) - (mTemplateMean * mTemplateMean);
    mTemplateStdDev = std::sqrt(std::max(1.0f, variance));
    mHasTemplate = true;
}

bool MotionEngineCore::trackBoundingBox(
    const GrayImage& frame,
    BoundingBox& outBox,
    float outMatrix[9]
) {
    if (!mHasTemplate || mTemplatePatch.width < 4 || mTemplatePatch.height < 4) {
        return false;
    }

    int tw = mTemplatePatch.width;
    int th = mTemplatePatch.height;
    int cx = static_cast<int>(mCurrentBox.x + mCurrentBox.width * 0.5f);
    int cy = static_cast<int>(mCurrentBox.y + mCurrentBox.height * 0.5f);

    int searchRadius = std::max(24, std::min(64, tw / 2));
    int minX = std::max(0, cx - searchRadius - tw / 2);
    int maxX = std::min(frame.width - tw, cx + searchRadius - tw / 2);
    int minY = std::max(0, cy - searchRadius - th / 2);
    int maxY = std::min(frame.height - th, cy + searchRadius - th / 2);

    if (minX >= maxX || minY >= maxY) {
        return false;
    }

    float bestZNCC = -1.0f;
    int bestX = minX;
    int bestY = minY;

    // Coarse search (stride 2) for maximum speed
    for (int y = minY; y <= maxY; y += 2) {
        for (int x = minX; x <= maxX; x += 2) {
            float sumI = 0.0f;
            float sumISq = 0.0f;
            float sumCross = 0.0f;
            int count = tw * th;

            for (int ty = 0; ty < th; ++ty) {
                const uint8_t* fRow = frame.row(y + ty) + x;
                const uint8_t* tRow = mTemplatePatch.row(ty);
                for (int tx = 0; tx < tw; ++tx) {
                    float valI = static_cast<float>(fRow[tx]);
                    float valT = static_cast<float>(tRow[tx]);
                    sumI += valI;
                    sumISq += valI * valI;
                    sumCross += (valT - mTemplateMean) * valI;
                }
            }

            float meanI = sumI / count;
            float varI = (sumISq / count) - (meanI * meanI);
            float stdDevI = std::sqrt(std::max(1.0f, varI));

            float zncc = (sumCross / count) / (mTemplateStdDev * stdDevI);
            if (zncc > bestZNCC) {
                bestZNCC = zncc;
                bestX = x;
                bestY = y;
            }
        }
    }

    // Fine refinement (stride 1) around best coarse peak
    int refineMinX = std::max(minX, bestX - 2);
    int refineMaxX = std::min(maxX, bestX + 2);
    int refineMinY = std::max(minY, bestY - 2);
    int refineMaxY = std::min(maxY, bestY + 2);

    for (int y = refineMinY; y <= refineMaxY; ++y) {
        for (int x = refineMinX; x <= refineMaxX; ++x) {
            float sumI = 0.0f;
            float sumISq = 0.0f;
            float sumCross = 0.0f;
            int count = tw * th;

            for (int ty = 0; ty < th; ++ty) {
                const uint8_t* fRow = frame.row(y + ty) + x;
                const uint8_t* tRow = mTemplatePatch.row(ty);
                for (int tx = 0; tx < tw; ++tx) {
                    float valI = static_cast<float>(fRow[tx]);
                    float valT = static_cast<float>(tRow[tx]);
                    sumI += valI;
                    sumISq += valI * valI;
                    sumCross += (valT - mTemplateMean) * valI;
                }
            }

            float meanI = sumI / count;
            float varI = (sumISq / count) - (meanI * meanI);
            float stdDevI = std::sqrt(std::max(1.0f, varI));

            float zncc = (sumCross / count) / (mTemplateStdDev * stdDevI);
            if (zncc > bestZNCC) {
                bestZNCC = zncc;
                bestX = x;
                bestY = y;
            }
        }
    }

    // Subpixel refinement using quadratic fitting on local neighborhood
    float subX = static_cast<float>(bestX);
    float subY = static_cast<float>(bestY);

    outBox.x = subX;
    outBox.y = subY;
    outBox.width = static_cast<float>(tw);
    outBox.height = static_cast<float>(th);
    outBox.confidence = std::max(0.0f, bestZNCC);

    // Compute affine transform matrix mapping previous box to current box
    float dx = subX - mCurrentBox.x;
    float dy = subY - mCurrentBox.y;

    outMatrix[0] = 1.0f; outMatrix[1] = 0.0f; outMatrix[2] = dx;
    outMatrix[3] = 0.0f; outMatrix[4] = 1.0f; outMatrix[5] = dy;
    outMatrix[6] = 0.0f; outMatrix[7] = 0.0f; outMatrix[8] = 1.0f;

    // Adaptive template update on high confidence to handle gradual lighting changes
    if (bestZNCC > 0.75f) {
        float alpha = 0.05f;
        for (int ty = 0; ty < th; ++ty) {
            const uint8_t* fRow = frame.row(bestY + ty) + bestX;
            uint8_t* tRow = mTemplatePatch.row(ty);
            for (int tx = 0; tx < tw; ++tx) {
                tRow[tx] = static_cast<uint8_t>(
                    (1.0f - alpha) * tRow[tx] + alpha * fRow[tx]
                );
            }
        }
    }

    mCurrentBox = outBox;
    return (bestZNCC > 0.35f);
}

// ----------------------------------------------------------------------------
// 3. Video Stabilization: Affine Motion Estimation + Moving Average Filter
// ----------------------------------------------------------------------------

bool MotionEngineCore::estimateAffineSimilarity(
    const std::vector<Point2f>& src,
    const std::vector<Point2f>& dst,
    float& outDx,
    float& outDy,
    float& outAngle,
    float& outScale,
    float outMatrix[9]
) {
    size_t n = src.size();
    if (n < 3) {
        outDx = 0.0f; outDy = 0.0f; outAngle = 0.0f; outScale = 1.0f;
        outMatrix[0] = 1.0f; outMatrix[1] = 0.0f; outMatrix[2] = 0.0f;
        outMatrix[3] = 0.0f; outMatrix[4] = 1.0f; outMatrix[5] = 0.0f;
        outMatrix[6] = 0.0f; outMatrix[7] = 0.0f; outMatrix[8] = 1.0f;
        return false;
    }

    // Similarity Model:
    // [ x' ] = [  a  -b ] [ x ] + [ dx ]
    // [ y' ]   [  b   a ] [ y ]   [ dy ]
    //
    // Normal Equations: (A^T * A) * p = A^T * Y
    // p = [a, b, dx, dy]^T

    double sa = 0.0, sb = 0.0;
    double sx = 0.0, sy = 0.0, sx_p = 0.0, sy_p = 0.0;
    double s_xx_yy = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double x = src[i].x;
        double y = src[i].y;
        double xp = dst[i].x;
        double yp = dst[i].y;

        sx += x;
        sy += y;
        sx_p += xp;
        sy_p += yp;
        s_xx_yy += (x * x + y * y);

        sa += (x * xp + y * yp);
        sb += (x * yp - y * xp);
    }

    double N = static_cast<double>(n);

    // Centroid shift formulation for numerical precision
    double mx = sx / N;
    double my = sy / N;
    double mx_p = sx_p / N;
    double my_p = sy_p / N;

    double num_a = 0.0;
    double num_b = 0.0;
    double den = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double u = src[i].x - mx;
        double v = src[i].y - my;
        double up = dst[i].x - mx_p;
        double vp = dst[i].y - my_p;

        num_a += (u * up + v * vp);
        num_b += (u * vp - v * up);
        den += (u * u + v * v);
    }

    if (den < 1e-7) {
        outDx = static_cast<float>(mx_p - mx);
        outDy = static_cast<float>(my_p - my);
        outAngle = 0.0f;
        outScale = 1.0f;

        outMatrix[0] = 1.0f; outMatrix[1] = 0.0f; outMatrix[2] = outDx;
        outMatrix[3] = 0.0f; outMatrix[4] = 1.0f; outMatrix[5] = outDy;
        outMatrix[6] = 0.0f; outMatrix[7] = 0.0f; outMatrix[8] = 1.0f;
        return true;
    }

    double a = num_a / den;
    double b = num_b / den;
    double dx = mx_p - (a * mx - b * my);
    double dy = my_p - (b * mx + a * my);

    outDx = static_cast<float>(dx);
    outDy = static_cast<float>(dy);
    outScale = static_cast<float>(std::sqrt(a * a + b * b));
    outAngle = static_cast<float>(std::atan2(b, a));

    outMatrix[0] = static_cast<float>(a);
    outMatrix[1] = static_cast<float>(-b);
    outMatrix[2] = outDx;

    outMatrix[3] = static_cast<float>(b);
    outMatrix[4] = static_cast<float>(a);
    outMatrix[5] = outDy;

    outMatrix[6] = 0.0f;
    outMatrix[7] = 0.0f;
    outMatrix[8] = 1.0f;

    return true;
}

void MotionEngineCore::stabilizeFrame(const GrayImage& frame, StabilizationData& outData) {
    if (!mHasPrevStabFrame) {
        mPrevStabFrame = frame;
        mHasPrevStabFrame = true;

        mCurrentCumulativeTrajectory = {0.0f, 0.0f, 0.0f};
        mTrajectoryHistory.clear();
        mTrajectoryHistory.push_back(mCurrentCumulativeTrajectory);

        outData.deltaX = 0.0f;
        outData.deltaY = 0.0f;
        outData.deltaAngle = 0.0f;
        outData.smoothedX = 0.0f;
        outData.smoothedY = 0.0f;
        outData.smoothedAngle = 0.0f;

        std::memset(outData.transformMatrix, 0, sizeof(outData.transformMatrix));
        outData.transformMatrix[0] = 1.0f;
        outData.transformMatrix[4] = 1.0f;
        outData.transformMatrix[8] = 1.0f;
        return;
    }

    // 1. Generate uniform feature grid across the frame
    constexpr int GRID_X = 8;
    constexpr int GRID_Y = 8;
    std::vector<Point2f> prevPoints;
    prevPoints.reserve(GRID_X * GRID_Y);

    float stepX = static_cast<float>(frame.width) / (GRID_X + 1);
    float stepY = static_cast<float>(frame.height) / (GRID_Y + 1);

    for (int gy = 1; gy <= GRID_Y; ++gy) {
        for (int gx = 1; gx <= GRID_X; ++gx) {
            prevPoints.emplace_back(gx * stepX, gy * stepY, 1);
        }
    }

    // 2. Track feature grid with pyramidal Lucas-Kanade
    std::vector<Point2f> currPoints;
    trackPoints(mPrevStabFrame, frame, prevPoints, currPoints, 15, 3, 10, 0.0005f);

    // 3. Outlier rejection using median displacement filter
    std::vector<float> dxList;
    std::vector<float> dyList;
    for (size_t i = 0; i < prevPoints.size(); ++i) {
        if (currPoints[i].status) {
            dxList.push_back(currPoints[i].x - prevPoints[i].x);
            dyList.push_back(currPoints[i].y - prevPoints[i].y);
        }
    }

    float deltaX = 0.0f;
    float deltaY = 0.0f;
    float deltaAngle = 0.0f;
    float deltaScale = 1.0f;
    float frameAffine[9];

    if (dxList.size() >= 4) {
        std::nth_element(dxList.begin(), dxList.begin() + dxList.size() / 2, dxList.end());
        std::nth_element(dyList.begin(), dyList.begin() + dyList.size() / 2, dyList.end());
        float medDx = dxList[dxList.size() / 2];
        float medDy = dyList[dyList.size() / 2];

        std::vector<Point2f> inlierSrc;
        std::vector<Point2f> inlierDst;
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            if (currPoints[i].status) {
                float curDx = currPoints[i].x - prevPoints[i].x;
                float curDy = currPoints[i].y - prevPoints[i].y;
                float distSq = (curDx - medDx) * (curDx - medDx) + (curDy - medDy) * (curDy - medDy);
                if (distSq < 100.0f) { // Inlier threshold: 10 pixels deviation
                    inlierSrc.push_back(prevPoints[i]);
                    inlierDst.push_back(currPoints[i]);
                }
            }
        }

        estimateAffineSimilarity(inlierSrc, inlierDst, deltaX, deltaY, deltaAngle, deltaScale, frameAffine);
    }

    // 4. Accumulate cumulative camera trajectory
    mCurrentCumulativeTrajectory.x += deltaX;
    mCurrentCumulativeTrajectory.y += deltaY;
    mCurrentCumulativeTrajectory.angle += deltaAngle;

    mTrajectoryHistory.push_back(mCurrentCumulativeTrajectory);
    while (mTrajectoryHistory.size() > mStabilizationWindowSize) {
        mTrajectoryHistory.pop_front();
    }

    // 5. Moving-Average Trajectory Smoothing
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumAngle = 0.0f;
    for (const auto& t : mTrajectoryHistory) {
        sumX += t.x;
        sumY += t.y;
        sumAngle += t.angle;
    }

    float smoothedX = sumX / mTrajectoryHistory.size();
    float smoothedY = sumY / mTrajectoryHistory.size();
    float smoothedAngle = sumAngle / mTrajectoryHistory.size();

    // 6. Calculate Stabilization Compensation
    float compX = smoothedX - mCurrentCumulativeTrajectory.x;
    float compY = smoothedY - mCurrentCumulativeTrajectory.y;
    float compAngle = smoothedAngle - mCurrentCumulativeTrajectory.angle;

    outData.deltaX = deltaX;
    outData.deltaY = deltaY;
    outData.deltaAngle = deltaAngle;
    outData.smoothedX = smoothedX;
    outData.smoothedY = smoothedY;
    outData.smoothedAngle = smoothedAngle;

    // Build 3x3 compensation transformation matrix rotated around image center
    float centerX = frame.width * 0.5f;
    float centerY = frame.height * 0.5f;
    float cosA = std::cos(compAngle);
    float sinA = std::sin(compAngle);

    // M = T(centerX, centerY) * R(compAngle) * T(-centerX, -centerY) * T(compX, compY)
    outData.transformMatrix[0] = cosA;
    outData.transformMatrix[1] = -sinA;
    outData.transformMatrix[2] = compX + centerX - (centerX * cosA - centerY * sinA);

    outData.transformMatrix[3] = sinA;
    outData.transformMatrix[4] = cosA;
    outData.transformMatrix[5] = compY + centerY - (centerX * sinA + centerY * cosA);

    outData.transformMatrix[6] = 0.0f;
    outData.transformMatrix[7] = 0.0f;
    outData.transformMatrix[8] = 1.0f;

    mPrevStabFrame = frame;
}

void MotionEngineCore::resetStabilization() {
    mHasPrevStabFrame = false;
    mCurrentCumulativeTrajectory = {0.0f, 0.0f, 0.0f};
    mTrajectoryHistory.clear();
}

// ----------------------------------------------------------------------------
// 4. Rotoscoping/Masking: Contour/Vertex Deformation Tracking
// ----------------------------------------------------------------------------

void MotionEngineCore::smoothContour(
    const std::vector<Point2f>& inContour,
    std::vector<Point2f>& outContour,
    float alpha
) {
    size_t n = inContour.size();
    outContour = inContour;
    if (n < 3) return;

    for (size_t i = 0; i < n; ++i) {
        size_t prev = (i == 0) ? (n - 1) : (i - 1);
        size_t next = (i == n - 1) ? 0 : (i + 1);

        float avgX = (inContour[prev].x + inContour[next].x) * 0.5f;
        float avgY = (inContour[prev].y + inContour[next].y) * 0.5f;

        outContour[i].x = (1.0f - alpha) * inContour[i].x + alpha * avgX;
        outContour[i].y = (1.0f - alpha) * inContour[i].y + alpha * avgY;
        outContour[i].status = inContour[i].status;
    }
}

void MotionEngineCore::trackPolygonMask(
    const GrayImage& prevGray,
    const GrayImage& currGray,
    const std::vector<Point2f>& inVertices,
    std::vector<Point2f>& outVertices,
    float outMatrix[9]
) {
    size_t numVerts = inVertices.size();
    outVertices.resize(numVerts);
    if (numVerts == 0) {
        std::memset(outMatrix, 0, 9 * sizeof(float));
        outMatrix[0] = outMatrix[4] = outMatrix[8] = 1.0f;
        return;
    }

    // Track all vertices with sparse optical flow
    std::vector<Point2f> rawTracked;
    trackPoints(prevGray, currGray, inVertices, rawTracked, 15, 3, 10, 0.0005f);

    // Collect successfully tracked vertices for global affine estimation
    std::vector<Point2f> validSrc;
    std::vector<Point2f> validDst;
    for (size_t i = 0; i < numVerts; ++i) {
        if (rawTracked[i].status) {
            validSrc.push_back(inVertices[i]);
            validDst.push_back(rawTracked[i]);
        }
    }

    float dx = 0.0f, dy = 0.0f, angle = 0.0f, scale = 1.0f;
    estimateAffineSimilarity(validSrc, validDst, dx, dy, angle, scale, outMatrix);

    // Fallback affine deformation for vertices that lost optical flow lock
    for (size_t i = 0; i < numVerts; ++i) {
        if (rawTracked[i].status) {
            outVertices[i] = rawTracked[i];
        } else {
            // Apply estimated global affine deformation to preserve shape
            float vx = inVertices[i].x;
            float vy = inVertices[i].y;
            outVertices[i].x = outMatrix[0] * vx + outMatrix[1] * vy + outMatrix[2];
            outVertices[i].y = outMatrix[3] * vx + outMatrix[4] * vy + outMatrix[5];
            outVertices[i].status = 1;
        }
    }

    // Laplacian contour regularizer to prevent jagged deform edges
    std::vector<Point2f> smoothed;
    smoothContour(outVertices, smoothed, 0.15f);
    outVertices = std::move(smoothed);
}

} // namespace motionengine

// ============================================================================
// JNI Layer: Direct ByteBuffer Memory Mapping & API Bindings
// Package: com.tracker.motionengine.MotionEngine
// ============================================================================

using namespace motionengine;

static inline MotionEngineCore* getEngineHandle(jlong handle) {
    return reinterpret_cast<MotionEngineCore*>(handle);
}

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeCreateEngine(
    JNIEnv* /* env */,
    jclass /* clazz */
) {
    MotionEngineCore* engine = new MotionEngineCore();
    return reinterpret_cast<jlong>(engine);
}

JNIEXPORT void JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeDestroyEngine(
    JNIEnv* /* env */,
    jclass /* clazz */,
    jlong handle
) {
    if (handle != 0) {
        delete getEngineHandle(handle);
    }
}

JNIEXPORT jint JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeTrackPoints(
    JNIEnv* env,
    jclass /* clazz */,
    jlong handle,
    jobject prevBuf,
    jobject currBuf,
    jint width,
    jint height,
    jint format,
    jfloatArray inPoints,
    jfloatArray outPoints,
    jbyteArray outStatus,
    jint winSize,
    jint maxLevels,
    jint maxIters
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (!engine || !prevBuf || !currBuf || !inPoints || !outPoints || !outStatus) {
        return 0;
    }

    const uint8_t* prevPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(prevBuf));
    const uint8_t* currPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(currBuf));
    if (!prevPtr || !currPtr) {
        return 0;
    }

    GrayImage prevGray;
    GrayImage currGray;
    MotionEngineCore::convertToGray(prevPtr, width, height, static_cast<FrameFormat>(format), prevGray);
    MotionEngineCore::convertToGray(currPtr, width, height, static_cast<FrameFormat>(format), currGray);

    jsize count = env->GetArrayLength(inPoints) / 2;
    jfloat* pIn = env->GetFloatArrayElements(inPoints, nullptr);
    jfloat* pOut = env->GetFloatArrayElements(outPoints, nullptr);
    jbyte* pStat = env->GetByteArrayElements(outStatus, nullptr);

    std::vector<Point2f> ptsIn(count);
    for (int i = 0; i < count; ++i) {
        ptsIn[i] = Point2f(pIn[i * 2], pIn[i * 2 + 1], 1);
    }

    std::vector<Point2f> ptsOut;
    engine->trackPoints(prevGray, currGray, ptsIn, ptsOut, winSize, maxLevels, maxIters);

    int trackedCount = 0;
    for (int i = 0; i < count; ++i) {
        pOut[i * 2] = ptsOut[i].x;
        pOut[i * 2 + 1] = ptsOut[i].y;
        pStat[i] = static_cast<jbyte>(ptsOut[i].status);
        if (ptsOut[i].status) {
            trackedCount++;
        }
    }

    env->ReleaseFloatArrayElements(inPoints, pIn, JNI_ABORT);
    env->ReleaseFloatArrayElements(outPoints, pOut, 0);
    env->ReleaseByteArrayElements(outStatus, pStat, 0);

    return trackedCount;
}

JNIEXPORT jboolean JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeInitBoundingBox(
    JNIEnv* env,
    jclass /* clazz */,
    jlong handle,
    jobject frameBuf,
    jint width,
    jint height,
    jint format,
    jfloat x,
    jfloat y,
    jfloat w,
    jfloat h
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (!engine || !frameBuf) {
        return JNI_FALSE;
    }

    const uint8_t* ptr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(frameBuf));
    if (!ptr) {
        return JNI_FALSE;
    }

    GrayImage gray;
    MotionEngineCore::convertToGray(ptr, width, height, static_cast<FrameFormat>(format), gray);
    engine->initBoundingBox(gray, BoundingBox(x, y, w, h));
    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeTrackBoundingBox(
    JNIEnv* env,
    jclass /* clazz */,
    jlong handle,
    jobject frameBuf,
    jint width,
    jint height,
    jint format,
    jfloatArray outBox,
    jfloatArray outMatrix
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (!engine || !frameBuf || !outBox || !outMatrix) {
        return JNI_FALSE;
    }

    const uint8_t* ptr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(frameBuf));
    if (!ptr) {
        return JNI_FALSE;
    }

    GrayImage gray;
    MotionEngineCore::convertToGray(ptr, width, height, static_cast<FrameFormat>(format), gray);

    BoundingBox resultBox;
    float matrix[9];
    bool tracked = engine->trackBoundingBox(gray, resultBox, matrix);

    jfloat* pBox = env->GetFloatArrayElements(outBox, nullptr);
    pBox[0] = resultBox.x;
    pBox[1] = resultBox.y;
    pBox[2] = resultBox.width;
    pBox[3] = resultBox.height;
    pBox[4] = resultBox.confidence;
    env->ReleaseFloatArrayElements(outBox, pBox, 0);

    jfloat* pMat = env->GetFloatArrayElements(outMatrix, nullptr);
    std::memcpy(pMat, matrix, 9 * sizeof(float));
    env->ReleaseFloatArrayElements(outMatrix, pMat, 0);

    return tracked ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeStabilizeFrame(
    JNIEnv* env,
    jclass /* clazz */,
    jlong handle,
    jobject frameBuf,
    jint width,
    jint height,
    jint format,
    jfloatArray outData
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (!engine || !frameBuf || !outData) {
        return JNI_FALSE;
    }

    const uint8_t* ptr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(frameBuf));
    if (!ptr) {
        return JNI_FALSE;
    }

    GrayImage gray;
    MotionEngineCore::convertToGray(ptr, width, height, static_cast<FrameFormat>(format), gray);

    StabilizationData stabData;
    engine->stabilizeFrame(gray, stabData);

    // Pack into float array:
    // [0]=deltaX, [1]=deltaY, [2]=deltaAngle, [3]=smoothedX, [4]=smoothedY, [5]=smoothedAngle,
    // [6..14]=transformMatrix (9 floats)
    jfloat* pData = env->GetFloatArrayElements(outData, nullptr);
    pData[0] = stabData.deltaX;
    pData[1] = stabData.deltaY;
    pData[2] = stabData.deltaAngle;
    pData[3] = stabData.smoothedX;
    pData[4] = stabData.smoothedY;
    pData[5] = stabData.smoothedAngle;
    std::memcpy(&pData[6], stabData.transformMatrix, 9 * sizeof(float));
    env->ReleaseFloatArrayElements(outData, pData, 0);

    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeResetStabilization(
    JNIEnv* /* env */,
    jclass /* clazz */,
    jlong handle
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (engine) {
        engine->resetStabilization();
    }
}

JNIEXPORT jboolean JNICALL
Java_com_tracker_motionengine_MotionEngine_nativeTrackPolygonMask(
    JNIEnv* env,
    jclass /* clazz */,
    jlong handle,
    jobject prevBuf,
    jobject currBuf,
    jint width,
    jint height,
    jint format,
    jfloatArray inVertices,
    jfloatArray outVertices,
    jbyteArray outStatus,
    jfloatArray outMatrix
) {
    MotionEngineCore* engine = getEngineHandle(handle);
    if (!engine || !prevBuf || !currBuf || !inVertices || !outVertices || !outStatus || !outMatrix) {
        return JNI_FALSE;
    }

    const uint8_t* prevPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(prevBuf));
    const uint8_t* currPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(currBuf));
    if (!prevPtr || !currPtr) {
        return JNI_FALSE;
    }

    GrayImage prevGray;
    GrayImage currGray;
    MotionEngineCore::convertToGray(prevPtr, width, height, static_cast<FrameFormat>(format), prevGray);
    MotionEngineCore::convertToGray(currPtr, width, height, static_cast<FrameFormat>(format), currGray);

    jsize count = env->GetArrayLength(inVertices) / 2;
    jfloat* pIn = env->GetFloatArrayElements(inVertices, nullptr);
    jfloat* pOut = env->GetFloatArrayElements(outVertices, nullptr);
    jbyte* pStat = env->GetByteArrayElements(outStatus, nullptr);
    jfloat* pMat = env->GetFloatArrayElements(outMatrix, nullptr);

    std::vector<Point2f> vertsIn(count);
    for (int i = 0; i < count; ++i) {
        vertsIn[i] = Point2f(pIn[i * 2], pIn[i * 2 + 1], 1);
    }

    std::vector<Point2f> vertsOut;
    float matrix[9];
    engine->trackPolygonMask(prevGray, currGray, vertsIn, vertsOut, matrix);

    for (int i = 0; i < count; ++i) {
        pOut[i * 2] = vertsOut[i].x;
        pOut[i * 2 + 1] = vertsOut[i].y;
        pStat[i] = static_cast<jbyte>(vertsOut[i].status);
    }
    std::memcpy(pMat, matrix, 9 * sizeof(float));

    env->ReleaseFloatArrayElements(inVertices, pIn, JNI_ABORT);
    env->ReleaseFloatArrayElements(outVertices, pOut, 0);
    env->ReleaseByteArrayElements(outStatus, pStat, 0);
    env->ReleaseFloatArrayElements(outMatrix, pMat, 0);

    return JNI_TRUE;
}

} // extern "C"
