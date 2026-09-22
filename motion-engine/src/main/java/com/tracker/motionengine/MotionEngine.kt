package com.tracker.motionengine

import java.nio.ByteBuffer

/**
 * High-performance, offline, classical computer-vision video editing motion engine.
 *
 * Implements pure algorithmic methods without AI or external dependencies:
 * - Sparse Optical Flow (Lucas-Kanade)
 * - Object Tracking (Zero-mean Normalized Cross-Correlation Template Matching)
 * - Video Stabilization (Affine Motion Estimation with Moving Average Trajectory Filter)
 * - Rotoscoping/Masking (Polygon Contour Vertex Deformation Tracking)
 */
class MotionEngine : AutoCloseable {

    companion object {
        init {
            System.loadLibrary("motionengine")
        }

        const val DEFAULT_WINDOW_SIZE = 15
        const val DEFAULT_MAX_LEVELS = 3
        const val DEFAULT_MAX_ITERS = 10
    }

    /**
     * Supported pixel formats for incoming frame buffers.
     */
    enum class FrameFormat(val value: Int) {
        RGBA(0),
        RGB(1),
        GRAYSCALE(2),
        NV21(3)
    }

    /**
     * 2D coordinate with tracking status.
     */
    data class Point(
        val x: Float,
        val y: Float,
        val isTracked: Boolean = true
    )

    /**
     * Bounding box representing an object's position, dimensions, and match confidence.
     */
    data class RectBox(
        val x: Float,
        val y: Float,
        val width: Float,
        val height: Float,
        val confidence: Float = 1.0f
    )

    /**
     * Video stabilization analysis results including inter-frame deltas, smoothed camera
     * trajectory, and the 3x3 compensation affine matrix (row-major).
     */
    data class StabilizationResult(
        val deltaX: Float,
        val deltaY: Float,
        val deltaAngle: Float,
        val smoothedX: Float,
        val smoothedY: Float,
        val smoothedAngle: Float,
        val transformMatrix: FloatArray
    ) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as StabilizationResult

            if (deltaX != other.deltaX) return false
            if (deltaY != other.deltaY) return false
            if (deltaAngle != other.deltaAngle) return false
            if (smoothedX != other.smoothedX) return false
            if (smoothedY != other.smoothedY) return false
            if (smoothedAngle != other.smoothedAngle) return false
            if (!transformMatrix.contentEquals(other.transformMatrix)) return false

            return true
        }

        override fun hashCode(): Int {
            var result = deltaX.hashCode()
            result = 31 * result + deltaY.hashCode()
            result = 31 * result + deltaAngle.hashCode()
            result = 31 * result + smoothedX.hashCode()
            result = 31 * result + smoothedY.hashCode()
            result = 31 * result + smoothedAngle.hashCode()
            result = 31 * result + transformMatrix.contentHashCode()
            return result
        }
    }

    /**
     * Polygon mask for video rotoscoping, containing vertex points and affine deformation matrix.
     */
    data class PolygonMask(
        val vertices: List<Point>,
        val transformMatrix: FloatArray
    ) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as PolygonMask

            if (vertices != other.vertices) return false
            if (!transformMatrix.contentEquals(other.transformMatrix)) return false

            return true
        }

        override fun hashCode(): Int {
            var result = vertices.hashCode()
            result = 31 * result + transformMatrix.contentHashCode()
            return result
        }
    }

    private var nativeHandle: Long = nativeCreateEngine()

    private fun checkActive() {
        check(nativeHandle != 0L) { "MotionEngine instance has already been closed and released." }
    }

    private fun validateBuffer(buffer: ByteBuffer, width: Int, height: Int, format: FrameFormat) {
        require(buffer.isDirect) { "ByteBuffer must be allocated as a direct buffer (ByteBuffer.allocateDirect)." }
        val minRequiredBytes = when (format) {
            FrameFormat.RGBA -> width * height * 4
            FrameFormat.RGB -> width * height * 3
            FrameFormat.GRAYSCALE -> width * height
            FrameFormat.NV21 -> (width * height * 3) / 2
        }
        require(buffer.capacity() >= minRequiredBytes) {
            "ByteBuffer capacity (${buffer.capacity()} bytes) is smaller than required minimum ($minRequiredBytes bytes) for $width x $height format $format."
        }
    }

    /**
     * Tracks discrete interest points from [prevFrame] to [currFrame] using Pyramidal Lucas-Kanade
     * sparse optical flow.
     *
     * @param prevFrame Direct ByteBuffer containing previous frame pixels.
     * @param currFrame Direct ByteBuffer containing current frame pixels.
     * @param width Frame pixel width.
     * @param height Frame pixel height.
     * @param points List of initial 2D points to track.
     * @param format Pixel buffer encoding (RGBA, RGB, GRAYSCALE, or NV21).
     * @param winSize Integration window size (e.g. 15 for 15x15).
     * @param maxLevels Pyramid levels for multi-scale tracking.
     * @param maxIters Maximum iterations per scale level.
     * @return List of tracked 2D points with updated coordinates and status flags.
     */
    fun trackPoints(
        prevFrame: ByteBuffer,
        currFrame: ByteBuffer,
        width: Int,
        height: Int,
        points: List<Point>,
        format: FrameFormat = FrameFormat.RGBA,
        winSize: Int = DEFAULT_WINDOW_SIZE,
        maxLevels: Int = DEFAULT_MAX_LEVELS,
        maxIters: Int = DEFAULT_MAX_ITERS
    ): List<Point> {
        checkActive()
        if (points.isEmpty()) return emptyList()

        validateBuffer(prevFrame, width, height, format)
        validateBuffer(currFrame, width, height, format)

        val inCoords = FloatArray(points.size * 2)
        for (i in points.indices) {
            inCoords[i * 2] = points[i].x
            inCoords[i * 2 + 1] = points[i].y
        }

        val outCoords = FloatArray(points.size * 2)
        val outStatus = ByteArray(points.size)

        nativeTrackPoints(
            nativeHandle,
            prevFrame,
            currFrame,
            width,
            height,
            format.value,
            inCoords,
            outCoords,
            outStatus,
            winSize,
            maxLevels,
            maxIters
        )

        val result = ArrayList<Point>(points.size)
        for (i in points.indices) {
            val px = outCoords[i * 2]
            val py = outCoords[i * 2 + 1]
            val isTracked = outStatus[i].toInt() != 0
            result.add(Point(px, py, isTracked))
        }

        return result
    }

    /**
     * Initializes the object template from a specified bounding box in the initial [frame].
     *
     * @param frame Direct ByteBuffer of the reference frame.
     * @param width Frame pixel width.
     * @param height Frame pixel height.
     * @param box Initial bounding box defining the object target.
     * @param format Pixel buffer encoding.
     */
    fun initBoundingBox(
        frame: ByteBuffer,
        width: Int,
        height: Int,
        box: RectBox,
        format: FrameFormat = FrameFormat.RGBA
    ): Boolean {
        checkActive()
        validateBuffer(frame, width, height, format)
        return nativeInitBoundingBox(
            nativeHandle,
            frame,
            width,
            height,
            format.value,
            box.x,
            box.y,
            box.width,
            box.height
        )
    }

    /**
     * Tracks the initialized bounding box across the new [frame] using zero-mean normalized
     * cross-correlation (ZNCC) with subpixel parabolic peak refinement.
     *
     * @param frame Direct ByteBuffer of the incoming frame.
     * @param width Frame pixel width.
     * @param height Frame pixel height.
     * @param format Pixel buffer encoding.
     * @return Pair containing the updated [RectBox] and the 3x3 transformation matrix.
     */
    fun trackBoundingBox(
        frame: ByteBuffer,
        width: Int,
        height: Int,
        format: FrameFormat = FrameFormat.RGBA
    ): Pair<RectBox, FloatArray> {
        checkActive()
        validateBuffer(frame, width, height, format)

        val outBox = FloatArray(5) // x, y, width, height, confidence
        val outMatrix = FloatArray(9)

        val success = nativeTrackBoundingBox(
            nativeHandle,
            frame,
            width,
            height,
            format.value,
            outBox,
            outMatrix
        )

        val trackedBox = RectBox(
            x = outBox[0],
            y = outBox[1],
            width = outBox[2],
            height = outBox[3],
            confidence = if (success) outBox[4] else 0.0f
        )

        return Pair(trackedBox, outMatrix)
    }

    /**
     * Stabilizes a video stream frame-by-frame by estimating affine camera movement (delta X,
     * delta Y, delta angle) and filtering cumulative camera trajectory with a temporal moving-average.
     *
     * @param frame Direct ByteBuffer containing the current frame.
     * @param width Frame pixel width.
     * @param height Frame pixel height.
     * @param format Pixel buffer encoding.
     * @return [StabilizationResult] containing motion vectors, smoothed trajectory, and 3x3 compensation matrix.
     */
    fun stabilizeFrame(
        frame: ByteBuffer,
        width: Int,
        height: Int,
        format: FrameFormat = FrameFormat.RGBA
    ): StabilizationResult {
        checkActive()
        validateBuffer(frame, width, height, format)

        val outData = FloatArray(15)
        nativeStabilizeFrame(
            nativeHandle,
            frame,
            width,
            height,
            format.value,
            outData
        )

        val matrix = FloatArray(9)
        System.arraycopy(outData, 6, matrix, 0, 9)

        return StabilizationResult(
            deltaX = outData[0],
            deltaY = outData[1],
            deltaAngle = outData[2],
            smoothedX = outData[3],
            smoothedY = outData[4],
            smoothedAngle = outData[5],
            transformMatrix = matrix
        )
    }

    /**
     * Resets the video stabilizer's cumulative trajectory history.
     */
    fun resetStabilization() {
        checkActive()
        nativeResetStabilization(nativeHandle)
    }

    /**
     * Tracks a rotoscoping polygon mask from [prevFrame] to [currFrame], deforming vertex
     * positions via optical flow and regularizing shape contour using Laplacian smoothing.
     *
     * @param prevFrame Direct ByteBuffer containing previous frame pixels.
     * @param currFrame Direct ByteBuffer containing current frame pixels.
     * @param width Frame pixel width.
     * @param height Frame pixel height.
     * @param mask Input polygon mask containing vertices to warp.
     * @param format Pixel buffer encoding.
     * @return Updated [PolygonMask] with warped vertices and deformation transform matrix.
     */
    fun trackPolygonMask(
        prevFrame: ByteBuffer,
        currFrame: ByteBuffer,
        width: Int,
        height: Int,
        mask: PolygonMask,
        format: FrameFormat = FrameFormat.RGBA
    ): PolygonMask {
        checkActive()
        if (mask.vertices.isEmpty()) return mask

        validateBuffer(prevFrame, width, height, format)
        validateBuffer(currFrame, width, height, format)

        val count = mask.vertices.size
        val inCoords = FloatArray(count * 2)
        for (i in 0 until count) {
            inCoords[i * 2] = mask.vertices[i].x
            inCoords[i * 2 + 1] = mask.vertices[i].y
        }

        val outCoords = FloatArray(count * 2)
        val outStatus = ByteArray(count)
        val outMatrix = FloatArray(9)

        nativeTrackPolygonMask(
            nativeHandle,
            prevFrame,
            currFrame,
            width,
            height,
            format.value,
            inCoords,
            outCoords,
            outStatus,
            outMatrix
        )

        val updatedVertices = ArrayList<Point>(count)
        for (i in 0 until count) {
            val px = outCoords[i * 2]
            val py = outCoords[i * 2 + 1]
            val isTracked = outStatus[i].toInt() != 0
            updatedVertices.add(Point(px, py, isTracked))
        }

        return PolygonMask(updatedVertices, outMatrix)
    }

    override fun close() {
        if (nativeHandle != 0L) {
            nativeDestroyEngine(nativeHandle)
            nativeHandle = 0L
        }
    }

    // ========================================================================
    // Native JNI Interface Declarations
    // ========================================================================

    private external fun nativeCreateEngine(): Long
    private external fun nativeDestroyEngine(handle: Long)

    private external fun nativeTrackPoints(
        handle: Long,
        prevBuf: ByteBuffer,
        currBuf: ByteBuffer,
        width: Int,
        height: Int,
        format: Int,
        inPoints: FloatArray,
        outPoints: FloatArray,
        outStatus: ByteArray,
        winSize: Int,
        maxLevels: Int,
        maxIters: Int
    ): Int

    private external fun nativeInitBoundingBox(
        handle: Long,
        frameBuf: ByteBuffer,
        width: Int,
        height: Int,
        format: Int,
        x: Float,
        y: Float,
        w: Float,
        h: Float
    ): Boolean

    private external fun nativeTrackBoundingBox(
        handle: Long,
        frameBuf: ByteBuffer,
        width: Int,
        height: Int,
        format: Int,
        outBox: FloatArray,
        outMatrix: FloatArray
    ): Boolean

    private external fun nativeStabilizeFrame(
        handle: Long,
        frameBuf: ByteBuffer,
        width: Int,
        height: Int,
        format: Int,
        outData: FloatArray
    ): Boolean

    private external fun nativeResetStabilization(handle: Long)

    private external fun nativeTrackPolygonMask(
        handle: Long,
        prevBuf: ByteBuffer,
        currBuf: ByteBuffer,
        width: Int,
        height: Int,
        format: Int,
        inVertices: FloatArray,
        outVertices: FloatArray,
        outStatus: ByteArray,
        outMatrix: FloatArray
    ): Boolean
}
