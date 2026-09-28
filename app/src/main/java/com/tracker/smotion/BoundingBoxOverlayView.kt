package com.tracker.smotion

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Interactive touch overlay that allows selecting, dragging, and resizing the tracking target
 * directly over the video preview with real-time video coordinate mapping.
 */
class BoundingBoxOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var videoWidth = 0
    private var videoHeight = 0

    // Video display render rectangle on the screen (accounting for letterbox/pillarbox)
    private val videoRenderRect = RectF()

    // Bounding box in video coordinates (0..videoWidth, 0..videoHeight)
    private val boxInVideoCoords = RectF(100f, 100f, 200f, 200f)

    // Bounding box transformed to screen view coordinates
    private val screenBoxRect = RectF()

    private var onBoxChangedListener: ((RectF) -> Unit)? = null

    // Touch interaction state
    private enum class TouchMode {
        NONE, MOVE, RESIZE_TL, RESIZE_TR, RESIZE_BL, RESIZE_BR, NEW_BOX
    }

    private var currentTouchMode = TouchMode.NONE
    private var lastTouchX = 0f
    private var lastTouchY = 0f
    private var newBoxStartX = 0f
    private var newBoxStartY = 0f

    private val handleRadius = 24f
    private val touchTolerance = 48f

    // Paints
    private val boxBorderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#00F2FE")
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private val cornerBracketPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#00F2FE")
        style = Paint.Style.STROKE
        strokeWidth = 6f
        strokeCap = Paint.Cap.ROUND
    }

    private val handleFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#4FACFE")
        style = Paint.Style.FILL
    }

    private val handleStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 2.5f
    }

    private val crosshairPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#00F2FE")
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }

    private val dimOverlayPaint = Paint().apply {
        color = Color.argb(90, 0, 0, 0)
        style = Paint.Style.FILL
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 28f
        isFakeBoldText = true
    }

    private val textBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(190, 15, 23, 42)
        style = Paint.Style.FILL
    }

    fun setVideoDimensions(w: Int, h: Int) {
        if (w <= 0 || h <= 0) return
        videoWidth = w
        videoHeight = h

        // Default initial box: centered at 20% of video dimensions
        val bw = (videoWidth * 0.20f).coerceAtLeast(60f)
        val bh = (videoHeight * 0.20f).coerceAtLeast(60f)
        val bx = (videoWidth - bw) * 0.5f
        val by = (videoHeight - bh) * 0.5f
        boxInVideoCoords.set(bx, by, bx + bw, by + bh)

        updateRenderRect()
        notifyBoxChanged()
        invalidate()
    }

    fun setPresetBox(fractionW: Float, fractionH: Float) {
        if (videoWidth <= 0 || videoHeight <= 0) return
        val bw = (videoWidth * fractionW).coerceAtLeast(40f)
        val bh = (videoHeight * fractionH).coerceAtLeast(40f)
        val cx = boxInVideoCoords.centerX()
        val cy = boxInVideoCoords.centerY()

        val left = (cx - bw * 0.5f).coerceIn(0f, (videoWidth - bw).coerceAtLeast(0f))
        val top = (cy - bh * 0.5f).coerceIn(0f, (videoHeight - bh).coerceAtLeast(0f))
        boxInVideoCoords.set(left, top, left + bw, top + bh)

        notifyBoxChanged()
        invalidate()
    }

    fun resetToCenter() {
        if (videoWidth <= 0 || videoHeight <= 0) return
        val bw = boxInVideoCoords.width().coerceAtLeast(60f)
        val bh = boxInVideoCoords.height().coerceAtLeast(60f)
        val left = (videoWidth - bw) * 0.5f
        val top = (videoHeight - bh) * 0.5f
        boxInVideoCoords.set(left, top, left + bw, top + bh)
        notifyBoxChanged()
        invalidate()
    }

    fun getSelectedBoxInVideoCoords(): RectF {
        return RectF(boxInVideoCoords)
    }

    fun setOnBoxChangedListener(listener: (RectF) -> Unit) {
        onBoxChangedListener = listener
    }

    private fun notifyBoxChanged() {
        onBoxChangedListener?.invoke(RectF(boxInVideoCoords))
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        updateRenderRect()
    }

    private fun updateRenderRect() {
        if (videoWidth <= 0 || videoHeight <= 0 || width <= 0 || height <= 0) {
            videoRenderRect.set(0f, 0f, width.toFloat(), height.toFloat())
            return
        }

        val videoAspect = videoWidth.toFloat() / videoHeight.toFloat()
        val viewAspect = width.toFloat() / height.toFloat()

        if (viewAspect > videoAspect) {
            // Pillarbox: video is narrower than view
            val renderW = height * videoAspect
            val left = (width - renderW) / 2f
            videoRenderRect.set(left, 0f, left + renderW, height.toFloat())
        } else {
            // Letterbox: video is wider than view
            val renderH = width / videoAspect
            val top = (height - renderH) / 2f
            videoRenderRect.set(0f, top, width.toFloat(), top + renderH)
        }
    }

    private fun videoToScreen(videoX: Float, videoY: Float): Pair<Float, Float> {
        if (videoWidth <= 0 || videoHeight <= 0) return Pair(videoX, videoY)
        val scaleX = videoRenderRect.width() / videoWidth.toFloat()
        val scaleY = videoRenderRect.height() / videoHeight.toFloat()
        val sx = videoRenderRect.left + videoX * scaleX
        val sy = videoRenderRect.top + videoY * scaleY
        return Pair(sx, sy)
    }

    private fun screenToVideo(screenX: Float, screenY: Float): Pair<Float, Float> {
        if (videoRenderRect.width() <= 0f || videoRenderRect.height() <= 0f) return Pair(screenX, screenY)
        val vx = (screenX - videoRenderRect.left) / videoRenderRect.width() * videoWidth
        val vy = (screenY - videoRenderRect.top) / videoRenderRect.height() * videoHeight
        return Pair(
            vx.coerceIn(0f, videoWidth.toFloat()),
            vy.coerceIn(0f, videoHeight.toFloat())
        )
    }

    private fun updateScreenBoxRect() {
        val (l, t) = videoToScreen(boxInVideoCoords.left, boxInVideoCoords.top)
        val (r, b) = videoToScreen(boxInVideoCoords.right, boxInVideoCoords.bottom)
        screenBoxRect.set(min(l, r), min(t, b), max(l, r), max(t, b))
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (videoWidth <= 0 || videoHeight <= 0) return false

        val x = event.x
        val y = event.y

        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                lastTouchX = x
                lastTouchY = y
                updateScreenBoxRect()

                // Check corner handles first for resizing
                currentTouchMode = when {
                    isNear(x, y, screenBoxRect.left, screenBoxRect.top) -> TouchMode.RESIZE_TL
                    isNear(x, y, screenBoxRect.right, screenBoxRect.top) -> TouchMode.RESIZE_TR
                    isNear(x, y, screenBoxRect.left, screenBoxRect.bottom) -> TouchMode.RESIZE_BL
                    isNear(x, y, screenBoxRect.right, screenBoxRect.bottom) -> TouchMode.RESIZE_BR
                    screenBoxRect.contains(x, y) -> TouchMode.MOVE
                    else -> {
                        newBoxStartX = x
                        newBoxStartY = y
                        TouchMode.NEW_BOX
                    }
                }
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                val dxScreen = x - lastTouchX
                val dyScreen = y - lastTouchY
                lastTouchX = x
                lastTouchY = y

                val scaleX = videoRenderRect.width() / videoWidth.toFloat()
                val scaleY = videoRenderRect.height() / videoHeight.toFloat()
                val dxVideo = if (scaleX > 0f) dxScreen / scaleX else 0f
                val dyVideo = if (scaleY > 0f) dyScreen / scaleY else 0f

                when (currentTouchMode) {
                    TouchMode.MOVE -> {
                        val w = boxInVideoCoords.width()
                        val h = boxInVideoCoords.height()
                        var newL = boxInVideoCoords.left + dxVideo
                        var newT = boxInVideoCoords.top + dyVideo
                        newL = newL.coerceIn(0f, videoWidth - w)
                        newT = newT.coerceIn(0f, videoHeight - h)
                        boxInVideoCoords.set(newL, newT, newL + w, newT + h)
                    }
                    TouchMode.RESIZE_TL -> {
                        boxInVideoCoords.left = (boxInVideoCoords.left + dxVideo).coerceIn(0f, boxInVideoCoords.right - 40f)
                        boxInVideoCoords.top = (boxInVideoCoords.top + dyVideo).coerceIn(0f, boxInVideoCoords.bottom - 40f)
                    }
                    TouchMode.RESIZE_TR -> {
                        boxInVideoCoords.right = (boxInVideoCoords.right + dxVideo).coerceIn(boxInVideoCoords.left + 40f, videoWidth.toFloat())
                        boxInVideoCoords.top = (boxInVideoCoords.top + dyVideo).coerceIn(0f, boxInVideoCoords.bottom - 40f)
                    }
                    TouchMode.RESIZE_BL -> {
                        boxInVideoCoords.left = (boxInVideoCoords.left + dxVideo).coerceIn(0f, boxInVideoCoords.right - 40f)
                        boxInVideoCoords.bottom = (boxInVideoCoords.bottom + dyVideo).coerceIn(boxInVideoCoords.top + 40f, videoHeight.toFloat())
                    }
                    TouchMode.RESIZE_BR -> {
                        boxInVideoCoords.right = (boxInVideoCoords.right + dxVideo).coerceIn(boxInVideoCoords.left + 40f, videoWidth.toFloat())
                        boxInVideoCoords.bottom = (boxInVideoCoords.bottom + dyVideo).coerceIn(boxInVideoCoords.top + 40f, videoHeight.toFloat())
                    }
                    TouchMode.NEW_BOX -> {
                        val (vStartX, vStartY) = screenToVideo(newBoxStartX, newBoxStartY)
                        val (vCurrX, vCurrY) = screenToVideo(x, y)
                        boxInVideoCoords.set(
                            min(vStartX, vCurrX),
                            min(vStartY, vCurrY),
                            max(vStartX, vCurrX),
                            max(vStartY, vCurrY)
                        )
                    }
                    else -> {}
                }

                notifyBoxChanged()
                invalidate()
                return true
            }

            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                // Ensure minimum dimension
                if (boxInVideoCoords.width() < 40f || boxInVideoCoords.height() < 40f) {
                    val cx = boxInVideoCoords.centerX()
                    val cy = boxInVideoCoords.centerY()
                    boxInVideoCoords.set(
                        (cx - 30f).coerceAtLeast(0f),
                        (cy - 30f).coerceAtLeast(0f),
                        (cx + 30f).coerceAtMost(videoWidth.toFloat()),
                        (cy + 30f).coerceAtMost(videoHeight.toFloat())
                    )
                }
                currentTouchMode = TouchMode.NONE
                notifyBoxChanged()
                invalidate()
                return true
            }
        }

        return super.onTouchEvent(event)
    }

    private fun isNear(x1: Float, y1: Float, x2: Float, y2: Float): Boolean {
        return abs(x1 - x2) <= touchTolerance && abs(y1 - y2) <= touchTolerance
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (videoWidth <= 0 || videoHeight <= 0) return

        updateScreenBoxRect()

        // 1. Draw dimming masks outside the selected target box
        canvas.drawRect(0f, 0f, width.toFloat(), screenBoxRect.top, dimOverlayPaint)
        canvas.drawRect(0f, screenBoxRect.bottom, width.toFloat(), height.toFloat(), dimOverlayPaint)
        canvas.drawRect(0f, screenBoxRect.top, screenBoxRect.left, screenBoxRect.bottom, dimOverlayPaint)
        canvas.drawRect(screenBoxRect.right, screenBoxRect.top, width.toFloat(), screenBoxRect.bottom, dimOverlayPaint)

        // 2. Draw Cyan bounding box frame
        canvas.drawRect(screenBoxRect, boxBorderPaint)

        // 3. Draw Corner Brackets (Visual targeting aesthetic)
        val bracketLen = min(28f, min(screenBoxRect.width(), screenBoxRect.height()) * 0.3f)
        val l = screenBoxRect.left
        val t = screenBoxRect.top
        val r = screenBoxRect.right
        val b = screenBoxRect.bottom

        // Top-Left
        canvas.drawLine(l, t, l + bracketLen, t, cornerBracketPaint)
        canvas.drawLine(l, t, l, t + bracketLen, cornerBracketPaint)
        // Top-Right
        canvas.drawLine(r, t, r - bracketLen, t, cornerBracketPaint)
        canvas.drawLine(r, t, r, t + bracketLen, cornerBracketPaint)
        // Bottom-Left
        canvas.drawLine(l, b, l + bracketLen, b, cornerBracketPaint)
        canvas.drawLine(l, b, l, b - bracketLen, cornerBracketPaint)
        // Bottom-Right
        canvas.drawLine(r, b, r - bracketLen, b, cornerBracketPaint)
        canvas.drawLine(r, b, r, b - bracketLen, cornerBracketPaint)

        // 4. Draw Center Crosshair Cursor
        val cx = screenBoxRect.centerX()
        val cy = screenBoxRect.centerY()
        val crossLen = 14f
        canvas.drawLine(cx - crossLen, cy, cx + crossLen, cy, crosshairPaint)
        canvas.drawLine(cx, cy - crossLen, cx, cy + crossLen, crosshairPaint)

        // 5. Draw 4 Corner Drag Handles
        drawHandle(canvas, l, t)
        drawHandle(canvas, r, t)
        drawHandle(canvas, l, b)
        drawHandle(canvas, r, b)

        // 6. Floating coordinate label
        val labelText = "Target: ${boxInVideoCoords.width().toInt()}x${boxInVideoCoords.height().toInt()} at (${boxInVideoCoords.left.toInt()}, ${boxInVideoCoords.top.toInt()})"
        val textWidth = textPaint.measureText(labelText)
        val labelX = screenBoxRect.left.coerceIn(12f, width - textWidth - 28f)
        val labelY = if (screenBoxRect.top > 60f) screenBoxRect.top - 14f else screenBoxRect.bottom + 36f

        val bgRect = RectF(labelX - 8f, labelY - 26f, labelX + textWidth + 8f, labelY + 8f)
        canvas.drawRoundRect(bgRect, 8f, 8f, textBgPaint)
        canvas.drawText(labelText, labelX, labelY, textPaint)
    }

    private fun drawHandle(canvas: Canvas, x: Float, y: Float) {
        canvas.drawCircle(x, y, handleRadius, handleFillPaint)
        canvas.drawCircle(x, y, handleRadius, handleStrokePaint)
    }
}
