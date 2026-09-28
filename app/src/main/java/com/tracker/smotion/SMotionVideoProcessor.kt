package com.tracker.smotion

import android.content.Context
import android.graphics.PointF
import android.graphics.RectF
import android.graphics.SurfaceTexture
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaExtractor
import android.media.MediaFormat
import android.media.MediaMetadataRetriever
import android.media.MediaMuxer
import android.net.Uri
import android.opengl.GLES20
import android.view.Surface
import com.tracker.motionengine.MotionEngine
import com.tracker.smotion.gles.EglCore
import com.tracker.smotion.gles.TextureRenderer
import com.tracker.smotion.gles.WindowSurface
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.exp
import kotlin.math.roundToInt

/**
 * High-performance offline video processor that tracks a target object using MotionEngine
 * and renders centered video frames via hardware-accelerated MediaCodec and OpenGL ES 2.0.
 */
class SMotionVideoProcessor(private val context: Context) {

    @Volatile
    private var isCancelled = false

    fun cancel() {
        isCancelled = true
    }

    suspend fun processVideo(
        inputUri: Uri,
        initialBox: RectF,
        zoom: Float = 1.2f,
        smoothness: Float = 0.40f,
        drawReticle: Boolean = true,
        outputFile: File = StorageHelper.createOutputFile(),
        progressCallback: (phase: String, progressPct: Int, currentFrame: Int, totalFrames: Int) -> Unit
    ): File = withContext(Dispatchers.IO) {
        isCancelled = false

        val retriever = MediaMetadataRetriever()
        try {
            retriever.setDataSource(context, inputUri)
        } catch (e: Exception) {
            throw IllegalArgumentException("Unable to read video from Uri: $inputUri", e)
        }

        val widthStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_WIDTH)
        val heightStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_HEIGHT)
        val durationStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)
        val rotationStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_ROTATION) ?: "0"

        val rawW = widthStr?.toIntOrNull() ?: 1280
        val rawH = heightStr?.toIntOrNull() ?: 720
        val rotation = rotationStr.toIntOrNull() ?: 0

        // Invert width and height if rotation is 90 or 270 degrees
        val isRotated = rotation == 90 || rotation == 270
        val inW = if (isRotated) rawH else rawW
        val inH = if (isRotated) rawW else rawH

        // Libx264 strictly requires even dimensions
        val outW = inW - (inW % 2)
        val outH = inH - (inH % 2)
        val durationMs = durationStr?.toLongOrNull() ?: 3000L

        retriever.release()

        // =====================================================================
        // PASS 1: Sequential Optical Flow & Object Tracking with MotionEngine
        // =====================================================================
        progressCallback("Pass 1/2: Tracking object across video...", 0, 0, 100)

        val rawCenters = ArrayList<PointF>()
        val trackW = 320
        val trackH = 180
        val trackBufSize = trackW * trackH * 4
        val trackDirectBuf = ByteBuffer.allocateDirect(trackBufSize).order(ByteOrder.nativeOrder())

        // Initial scaled bounding box
        val scaleX = trackW.toFloat() / inW.toFloat()
        val scaleY = trackH.toFloat() / inH.toFloat()
        val scaledBox = MotionEngine.RectBox(
            x = initialBox.left * scaleX,
            y = initialBox.top * scaleY,
            width = initialBox.width() * scaleX,
            height = initialBox.height() * scaleY
        )

        // Setup Pass 1 Offscreen Decoder
        val extractor1 = MediaExtractor()
        extractor1.setDataSource(context, inputUri, null)
        val videoTrack1 = findTrack(extractor1, "video/")
        check(videoTrack1 >= 0) { "No video track found in input file" }
        extractor1.selectTrack(videoTrack1)
        val videoFormat1 = extractor1.getTrackFormat(videoTrack1)
        val mime1 = videoFormat1.getString(MediaFormat.KEY_MIME) ?: "video/avc"

        val eglCore1 = EglCore(null, 0)
        // Offscreen WindowSurface for Pass 1 frame reading
        val pbuffer1 = WindowSurface(eglCore1, Surface(SurfaceTexture(0).apply { setDefaultBufferSize(trackW, trackH) }), releaseSurface = true)
        pbuffer1.makeCurrent()

        val renderer1 = TextureRenderer()
        renderer1.surfaceCreated()
        val surfaceTexture1 = SurfaceTexture(renderer1.textureId)
        surfaceTexture1.setDefaultBufferSize(trackW, trackH)
        val decoderSurface1 = Surface(surfaceTexture1)

        val decoder1 = MediaCodec.createDecoderByType(mime1)
        decoder1.configure(videoFormat1, decoderSurface1, null, 0)
        decoder1.start()

        val texMatrix1 = FloatArray(16)
        val identMatrix1 = FloatArray(16)
        android.opengl.Matrix.setIdentityM(identMatrix1, 0)

        var totalFramesEstimate = ((durationMs / 1000.0) * 30.0).roundToInt().coerceAtLeast(30)

        MotionEngine().use { engine ->
            val bufferInfo = MediaCodec.BufferInfo()
            var inputEos1 = false
            var outputEos1 = false
            var frameIndex1 = 0

            while (!outputEos1 && !isCancelled) {
                if (!inputEos1) {
                    val inIdx = decoder1.dequeueInputBuffer(10000L)
                    if (inIdx >= 0) {
                        val inBuf = decoder1.getInputBuffer(inIdx)
                        if (inBuf != null) {
                            val sampleSize = extractor1.readSampleData(inBuf, 0)
                            if (sampleSize < 0) {
                                decoder1.queueInputBuffer(inIdx, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                                inputEos1 = true
                            } else {
                                val pts = extractor1.sampleTime
                                decoder1.queueInputBuffer(inIdx, 0, sampleSize, pts, 0)
                                extractor1.advance()
                            }
                        }
                    }
                }

                val outIdx = decoder1.dequeueOutputBuffer(bufferInfo, 10000L)
                if (outIdx >= 0) {
                    if ((bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        outputEos1 = true
                    }
                    val render = bufferInfo.size > 0
                    decoder1.releaseOutputBuffer(outIdx, render)

                    if (render) {
                        surfaceTexture1.updateTexImage()
                        surfaceTexture1.getTransformMatrix(texMatrix1)

                        // Render to offscreen buffer and read pixels into direct buffer
                        GLES20.glViewport(0, 0, trackW, trackH)
                        renderer1.drawFrame(texMatrix1, identMatrix1, false)

                        trackDirectBuf.position(0)
                        GLES20.glReadPixels(0, 0, trackW, trackH, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, trackDirectBuf)

                        trackDirectBuf.position(0)
                        if (frameIndex1 == 0) {
                            engine.initBoundingBox(trackDirectBuf, trackW, trackH, scaledBox, MotionEngine.FrameFormat.RGBA)
                            val initCx = initialBox.centerX()
                            val initCy = initialBox.centerY()
                            rawCenters.add(PointF(initCx, initCy))
                        } else {
                            val (trackedBox, _) = engine.trackBoundingBox(trackDirectBuf, trackW, trackH, MotionEngine.FrameFormat.RGBA)
                            val cx = (trackedBox.x + trackedBox.width * 0.5f) / scaleX
                            val cy = (trackedBox.y + trackedBox.height * 0.5f) / scaleY
                            rawCenters.add(PointF(cx.coerceIn(0f, inW.toFloat()), cy.coerceIn(0f, inH.toFloat())))
                        }

                        frameIndex1++
                        if (frameIndex1 % 5 == 0) {
                            val pct = ((frameIndex1.toFloat() / totalFramesEstimate.toFloat()) * 50f).roundToInt().coerceIn(0, 50)
                            progressCallback("Pass 1/2: Subpixel Tracking ($frameIndex1 frames)", pct, frameIndex1, totalFramesEstimate)
                        }
                    }
                }
            }
        }

        decoder1.stop()
        decoder1.release()
        decoderSurface1.release()
        surfaceTexture1.release()
        pbuffer1.release()
        eglCore1.release()
        extractor1.release()

        if (isCancelled) {
            outputFile.delete()
            throw InterruptedException("Export cancelled by user")
        }

        val totalFrames = rawCenters.size
        check(totalFrames > 0) { "Could not extract or track any frames from the input video." }

        // =====================================================================
        // Zero-Phase Gaussian Trajectory Smoothing Filter
        // =====================================================================
        val smoothedCenters = smoothTrajectoryGaussian(rawCenters, smoothness)

        // =====================================================================
        // PASS 2: Hardware MediaCodec Centering & H.264 Encoding with Audio Muxing
        // =====================================================================
        progressCallback("Pass 2/2: Hardware MediaCodec Centering & Encoding...", 50, 0, totalFrames)

        val extractor2 = MediaExtractor()
        extractor2.setDataSource(context, inputUri, null)
        val videoTrack2 = findTrack(extractor2, "video/")
        val audioTrack2 = findTrack(extractor2, "audio/")
        extractor2.selectTrack(videoTrack2)
        val videoFormat2 = extractor2.getTrackFormat(videoTrack2)
        val mime2 = videoFormat2.getString(MediaFormat.KEY_MIME) ?: "video/avc"

        val fps = if (videoFormat2.containsKey(MediaFormat.KEY_FRAME_RATE)) {
            videoFormat2.getInteger(MediaFormat.KEY_FRAME_RATE).coerceIn(15, 60)
        } else {
            30
        }

        val bitRate = ((outW * outH * 3.5f).toInt()).coerceIn(3_000_000, 16_000_000)

        // Setup MediaMuxer
        val muxer = MediaMuxer(outputFile.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)

        // Setup Encoder
        val encFormat = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, outW, outH).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, bitRate)
            setInteger(MediaFormat.KEY_FRAME_RATE, fps)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
        }

        val encoder = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
        encoder.configure(encFormat, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        val encoderInputSurface = encoder.createInputSurface()
        encoder.start()

        val eglCore2 = EglCore(null, EglCore.FLAG_RECORDABLE)
        val windowSurface2 = WindowSurface(eglCore2, encoderInputSurface, true)
        windowSurface2.makeCurrent()

        val renderer2 = TextureRenderer()
        renderer2.surfaceCreated()
        val surfaceTexture2 = SurfaceTexture(renderer2.textureId)
        surfaceTexture2.setDefaultBufferSize(outW, outH)
        val decoderSurface2 = Surface(surfaceTexture2)

        val decoder2 = MediaCodec.createDecoderByType(mime2)
        decoder2.configure(videoFormat2, decoderSurface2, null, 0)
        decoder2.start()

        val texMatrix2 = FloatArray(16)
        val mvpMatrix2 = FloatArray(16)
        val encBufferInfo = MediaCodec.BufferInfo()
        val decBufferInfo = MediaCodec.BufferInfo()

        var muxerStarted = false
        var muxerVideoTrackIdx = -1
        var muxerAudioTrackIdx = -1

        var inputEos2 = false
        var outputEos2 = false
        var frameIndex2 = 0

        try {
            while (!outputEos2 && !isCancelled) {
                // Feed decoder
                if (!inputEos2) {
                    val inIdx = decoder2.dequeueInputBuffer(10000L)
                    if (inIdx >= 0) {
                        val inBuf = decoder2.getInputBuffer(inIdx)
                        if (inBuf != null) {
                            val sampleSize = extractor2.readSampleData(inBuf, 0)
                            if (sampleSize < 0) {
                                decoder2.queueInputBuffer(inIdx, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                                inputEos2 = true
                            } else {
                                val pts = extractor2.sampleTime
                                decoder2.queueInputBuffer(inIdx, 0, sampleSize, pts, 0)
                                extractor2.advance()
                            }
                        }
                    }
                }

                // Decode output -> Render centered to encoder surface
                val outIdx = decoder2.dequeueOutputBuffer(decBufferInfo, 10000L)
                if (outIdx >= 0) {
                    if ((decBufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        outputEos2 = true
                        encoder.signalEndOfInputStream()
                    }

                    val render = decBufferInfo.size > 0
                    decoder2.releaseOutputBuffer(outIdx, render)

                    if (render && frameIndex2 < totalFrames) {
                        surfaceTexture2.updateTexImage()
                        surfaceTexture2.getTransformMatrix(texMatrix2)

                        val targetCenter = smoothedCenters[frameIndex2]
                        renderer2.computeCenteringMvpMatrix(inW, inH, targetCenter.x, targetCenter.y, zoom, mvpMatrix2)

                        windowSurface2.makeCurrent()
                        GLES20.glViewport(0, 0, outW, outH)
                        renderer2.drawFrame(texMatrix2, mvpMatrix2, drawReticle)

                        val ptsNs = decBufferInfo.presentationTimeUs * 1000L
                        windowSurface2.setPresentationTime(ptsNs)
                        windowSurface2.swapBuffers()

                        frameIndex2++
                        if (frameIndex2 % 5 == 0 || frameIndex2 == totalFrames) {
                            val pct = 50 + ((frameIndex2.toFloat() / totalFrames.toFloat()) * 50f).roundToInt().coerceIn(0, 50)
                            progressCallback("Pass 2/2: Encoding centered frame $frameIndex2/$totalFrames", pct, frameIndex2, totalFrames)
                        }
                    }
                }

                // Drain encoder
                var encOutIdx = encoder.dequeueOutputBuffer(encBufferInfo, 10000L)
                while (encOutIdx >= 0) {
                    if ((encBufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
                        encBufferInfo.size = 0
                    }

                    if (encBufferInfo.size > 0 && muxerStarted) {
                        val encBuf = encoder.getOutputBuffer(encOutIdx)
                        if (encBuf != null) {
                            encBuf.position(encBufferInfo.offset)
                            encBuf.limit(encBufferInfo.offset + encBufferInfo.size)
                            muxer.writeSampleData(muxerVideoTrackIdx, encBuf, encBufferInfo)
                        }
                    }

                    val isEos = (encBufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0
                    encoder.releaseOutputBuffer(encOutIdx, false)
                    if (isEos) break
                    encOutIdx = encoder.dequeueOutputBuffer(encBufferInfo, 0L)
                }

                if (encOutIdx == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    val newFormat = encoder.outputFormat
                    muxerVideoTrackIdx = muxer.addTrack(newFormat)

                    // Add Audio Track if available
                    if (audioTrack2 >= 0) {
                        val audioFormat = extractor2.getTrackFormat(audioTrack2)
                        muxerAudioTrackIdx = muxer.addTrack(audioFormat)
                    }

                    muxer.start()
                    muxerStarted = true
                }
            }

            // Drain remaining encoder frames
            var draining = true
            while (draining && !isCancelled) {
                val encOutIdx = encoder.dequeueOutputBuffer(encBufferInfo, 20000L)
                if (encOutIdx >= 0) {
                    if (encBufferInfo.size > 0 && muxerStarted) {
                        val encBuf = encoder.getOutputBuffer(encOutIdx)
                        if (encBuf != null) {
                            encBuf.position(encBufferInfo.offset)
                            encBuf.limit(encBufferInfo.offset + encBufferInfo.size)
                            muxer.writeSampleData(muxerVideoTrackIdx, encBuf, encBufferInfo)
                        }
                    }
                    val isEos = (encBufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0
                    encoder.releaseOutputBuffer(encOutIdx, false)
                    if (isEos) draining = false
                } else if (encOutIdx == MediaCodec.INFO_TRY_AGAIN_LATER) {
                    draining = false
                }
            }

            // Copy audio samples directly to preserve original audio losslessly
            if (audioTrack2 >= 0 && muxerStarted && muxerAudioTrackIdx >= 0 && !isCancelled) {
                extractor2.unselectTrack(videoTrack2)
                extractor2.selectTrack(audioTrack2)
                extractor2.seekTo(0L, MediaExtractor.SEEK_TO_PREVIOUS_SYNC)

                val audioBuf = ByteBuffer.allocateDirect(128 * 1024)
                val audioBufferInfo = MediaCodec.BufferInfo()

                while (!isCancelled) {
                    val sampleSize = extractor2.readSampleData(audioBuf, 0)
                    if (sampleSize < 0) break

                    audioBufferInfo.offset = 0
                    audioBufferInfo.size = sampleSize
                    audioBufferInfo.presentationTimeUs = extractor2.sampleTime
                    audioBufferInfo.flags = extractor2.sampleFlags

                    muxer.writeSampleData(muxerAudioTrackIdx, audioBuf, audioBufferInfo)
                    extractor2.advance()
                }
            }

        } finally {
            try {
                if (muxerStarted) {
                    muxer.stop()
                }
            } catch (e: Exception) {
                // Ignore muxer stop errors if cancelled
            }
            muxer.release()

            decoder2.stop()
            decoder2.release()
            encoder.stop()
            encoder.release()
            decoderSurface2.release()
            surfaceTexture2.release()
            windowSurface2.release()
            eglCore2.release()
            extractor2.release()
        }

        if (isCancelled) {
            outputFile.delete()
            throw InterruptedException("Export cancelled by user")
        }

        // Register with Android MediaStore
        StorageHelper.scanMediaFile(context, outputFile)

        progressCallback("Complete! Saved to /Movies/SMotion", 100, totalFrames, totalFrames)
        return@withContext outputFile
    }

    private fun findTrack(extractor: MediaExtractor, prefix: String): Int {
        for (i in 0 until extractor.trackCount) {
            val format = extractor.getTrackFormat(i)
            val mime = format.getString(MediaFormat.KEY_MIME) ?: ""
            if (mime.startsWith(prefix)) {
                return i
            }
        }
        return -1
    }

    private fun smoothTrajectoryGaussian(trajectory: List<PointF>, smooth: Float): List<PointF> {
        val n = trajectory.size
        if (n <= 1) return trajectory

        val clamped = smooth.coerceIn(0.05f, 1.0f)
        val radius = ((1.05f - clamped) * 26f).roundToInt().coerceIn(2, 32)
        val sigma = (radius / 2.2f).coerceAtLeast(1.0f)

        val weights = FloatArray(radius * 2 + 1) { k ->
            val d = (k - radius).toFloat()
            exp(-d * d / (2.0f * sigma * sigma))
        }

        return List(n) { i ->
            var wSum = 0f
            var valX = 0f
            var valY = 0f
            for (k in -radius..radius) {
                val idx = (i + k).coerceIn(0, n - 1)
                val w = weights[k + radius]
                valX += trajectory[idx].x * w
                valY += trajectory[idx].y * w
                wSum += w
            }
            PointF(valX / wSum, valY / wSum)
        }
    }
}
