package com.tracker.smotion.gles

import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.opengl.Matrix
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * OpenGL ES 2.0 renderer that renders decoded OES video frames with an affine transformation
 * matrix to smoothly center the tracked object on the output canvas with zoom.
 */
class TextureRenderer {

    companion object {
        private const val FLOAT_SIZE_BYTES = 4
        private const val TRIANGLE_VERTICES_DATA_STRIDE_BYTES = 5 * FLOAT_SIZE_BYTES
        private const val TRIANGLE_VERTICES_DATA_POS_OFFSET = 0
        private const val TRIANGLE_VERTICES_DATA_UV_OFFSET = 3

        private val QUAD_VERTICES_DATA = floatArrayOf(
            // X, Y, Z, U, V
            -1.0f, -1.0f, 0f, 0f, 0f,
             1.0f, -1.0f, 0f, 1f, 0f,
            -1.0f,  1.0f, 0f, 0f, 1f,
             1.0f,  1.0f, 0f, 1f, 1f
        )

        private const val VERTEX_SHADER = """
            uniform mat4 uMVPMatrix;
            uniform mat4 uTexMatrix;
            attribute vec4 aPosition;
            attribute vec4 aTextureCoord;
            varying vec2 vTextureCoord;
            void main() {
                gl_Position = uMVPMatrix * aPosition;
                vTextureCoord = (uTexMatrix * aTextureCoord).xy;
            }
        """

        private const val FRAGMENT_SHADER = """
            #extension GL_OES_EGL_image_external : require
            precision mediump float;
            varying vec2 vTextureCoord;
            uniform samplerExternalOES sTexture;
            void main() {
                gl_FragColor = texture2D(sTexture, vTextureCoord);
            }
        """

        // Reticle shader for center targeting overlay
        private const val RETICLE_VERTEX_SHADER = """
            uniform mat4 uMVPMatrix;
            attribute vec4 aPosition;
            void main() {
                gl_Position = uMVPMatrix * aPosition;
            }
        """

        private const val RETICLE_FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec4 uColor;
            void main() {
                gl_FragColor = uColor;
            }
        """
    }

    private val triangleVertices: FloatBuffer = ByteBuffer.allocateDirect(
        QUAD_VERTICES_DATA.size * FLOAT_SIZE_BYTES
    ).order(ByteOrder.nativeOrder()).asFloatBuffer().apply {
        put(QUAD_VERTICES_DATA)
        position(0)
    }

    var textureId: Int = -1
        private set

    private var program = 0
    private var uMVPMatrixLoc = 0
    private var uTexMatrixLoc = 0
    private var aPositionLoc = 0
    private var aTextureCoordLoc = 0

    // Reticle program
    private var reticleProgram = 0
    private var reticleMVPMatrixLoc = 0
    private var reticleColorLoc = 0
    private var reticlePositionLoc = 0

    private val reticleVertices: FloatBuffer = ByteBuffer.allocateDirect(
        16 * 2 * FLOAT_SIZE_BYTES
    ).order(ByteOrder.nativeOrder()).asFloatBuffer()

    fun surfaceCreated() {
        program = createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        check(program != 0) { "failed creating OES texture program" }

        aPositionLoc = GLES20.glGetAttribLocation(program, "aPosition")
        aTextureCoordLoc = GLES20.glGetAttribLocation(program, "aTextureCoord")
        uMVPMatrixLoc = GLES20.glGetUniformLocation(program, "uMVPMatrix")
        uTexMatrixLoc = GLES20.glGetUniformLocation(program, "uTexMatrix")

        // Reticle program
        reticleProgram = createProgram(RETICLE_VERTEX_SHADER, RETICLE_FRAGMENT_SHADER)
        if (reticleProgram != 0) {
            reticlePositionLoc = GLES20.glGetAttribLocation(reticleProgram, "aPosition")
            reticleMVPMatrixLoc = GLES20.glGetUniformLocation(reticleProgram, "uMVPMatrix")
            reticleColorLoc = GLES20.glGetUniformLocation(reticleProgram, "uColor")
        }

        // Generate external texture
        val textures = IntArray(1)
        GLES20.glGenTextures(1, textures, 0)
        textureId = textures[0]
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glTexParameterf(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR.toFloat())
        GLES20.glTexParameterf(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR.toFloat())
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
    }

    fun computeCenteringMvpMatrix(
        inW: Int, inH: Int,
        targetX: Float, targetY: Float,
        zoom: Float,
        outMvpMatrix: FloatArray
    ) {
        Matrix.setIdentityM(outMvpMatrix, 0)

        // Normalize target point to [-1.0, 1.0] OpenGL NDC space
        // Note: in video coordinates (0, 0) is top-left, while OpenGL (0, 0) is center and +1 is top.
        val normX = (targetX / inW.toFloat()) * 2.0f - 1.0f
        val normY = 1.0f - (targetY / inH.toFloat()) * 2.0f

        // Translate so that (normX, normY) ends up at (0, 0)
        Matrix.translateM(outMvpMatrix, 0, -normX * zoom, -normY * zoom, 0f)

        // Scale by zoom
        Matrix.scaleM(outMvpMatrix, 0, zoom, zoom, 1.0f)
    }

    fun drawFrame(
        texMatrix: FloatArray,
        mvpMatrix: FloatArray,
        drawReticle: Boolean = true
    ) {
        // Clear background with dark aesthetic
        GLES20.glClearColor(0.027f, 0.043f, 0.078f, 1.0f)
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)

        GLES20.glUseProgram(program)

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)

        triangleVertices.position(TRIANGLE_VERTICES_DATA_POS_OFFSET)
        GLES20.glVertexAttribPointer(
            aPositionLoc, 3, GLES20.GL_FLOAT, false,
            TRIANGLE_VERTICES_DATA_STRIDE_BYTES, triangleVertices
        )
        GLES20.glEnableVertexAttribArray(aPositionLoc)

        triangleVertices.position(TRIANGLE_VERTICES_DATA_UV_OFFSET)
        GLES20.glVertexAttribPointer(
            aTextureCoordLoc, 2, GLES20.GL_FLOAT, false,
            TRIANGLE_VERTICES_DATA_STRIDE_BYTES, triangleVertices
        )
        GLES20.glEnableVertexAttribArray(aTextureCoordLoc)

        GLES20.glUniformMatrix4fv(uMVPMatrixLoc, 1, false, mvpMatrix, 0)
        GLES20.glUniformMatrix4fv(uTexMatrixLoc, 1, false, texMatrix, 0)

        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)

        GLES20.glDisableVertexAttribArray(aPositionLoc)
        GLES20.glDisableVertexAttribArray(aTextureCoordLoc)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, 0)

        // Draw Center Reticle Overlay if requested
        if (drawReticle && reticleProgram != 0) {
            drawCenterReticle()
        }
    }

    private fun drawCenterReticle() {
        GLES20.glUseProgram(reticleProgram)

        val ident = FloatArray(16)
        Matrix.setIdentityM(ident, 0)
        GLES20.glUniformMatrix4fv(reticleMVPMatrixLoc, 1, false, ident, 0)

        // Vibrant Cyan Color (#00F2FE) with slight transparency
        GLES20.glUniform4f(reticleColorLoc, 0.0f, 0.949f, 0.996f, 0.85f)

        val crossLen = 0.06f
        val gap = 0.015f
        val lines = floatArrayOf(
            // Horizontal crosshairs
            -crossLen, 0f, -gap, 0f,
             gap, 0f,  crossLen, 0f,
            // Vertical crosshairs
            0f, -crossLen, 0f, -gap,
            0f,  gap, 0f,  crossLen
        )

        reticleVertices.clear()
        reticleVertices.put(lines)
        reticleVertices.position(0)

        GLES20.glVertexAttribPointer(reticlePositionLoc, 2, GLES20.GL_FLOAT, false, 2 * FLOAT_SIZE_BYTES, reticleVertices)
        GLES20.glEnableVertexAttribArray(reticlePositionLoc)

        GLES20.glLineWidth(3.0f)
        GLES20.glDrawArrays(GLES20.GL_LINES, 0, lines.size / 2)

        GLES20.glDisableVertexAttribArray(reticlePositionLoc)
    }

    private fun loadShader(shaderType: Int, source: String): Int {
        var shader = GLES20.glCreateShader(shaderType)
        if (shader != 0) {
            GLES20.glShaderSource(shader, source)
            GLES20.glCompileShader(shader)
            val compiled = IntArray(1)
            GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, compiled, 0)
            if (compiled[0] == 0) {
                GLES20.glDeleteShader(shader)
                shader = 0
            }
        }
        return shader
    }

    private fun createProgram(vertexSource: String, fragmentSource: String): Int {
        val vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexSource)
        if (vertexShader == 0) return 0
        val pixelShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentSource)
        if (pixelShader == 0) return 0

        var prog = GLES20.glCreateProgram()
        if (prog != 0) {
            GLES20.glAttachShader(prog, vertexShader)
            GLES20.glAttachShader(prog, pixelShader)
            GLES20.glLinkProgram(prog)
            val linkStatus = IntArray(1)
            GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, linkStatus, 0)
            if (linkStatus[0] != GLES20.GL_TRUE) {
                GLES20.glDeleteProgram(prog)
                prog = 0
            }
        }
        return prog
    }
}
