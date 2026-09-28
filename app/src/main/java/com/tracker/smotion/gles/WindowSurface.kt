package com.tracker.smotion.gles

import android.opengl.EGL14
import android.opengl.EGLSurface
import android.view.Surface

/**
 * Recordable EGL surface wrapper backed by an Android Surface (e.g. MediaCodec input surface).
 */
class WindowSurface(
    private val eglCore: EglCore,
    surface: Surface,
    private val releaseSurface: Boolean = false
) {
    private var eglSurface: EGLSurface = eglCore.createWindowSurface(surface)
    private var surfaceRef: Surface? = surface

    fun release() {
        eglCore.releaseSurface(eglSurface)
        eglSurface = EGL14.EGL_NO_SURFACE
        if (releaseSurface && surfaceRef != null) {
            surfaceRef?.release()
            surfaceRef = null
        }
    }

    fun makeCurrent() {
        eglCore.makeCurrent(eglSurface)
    }

    fun swapBuffers(): Boolean {
        return eglCore.swapBuffers(eglSurface)
    }

    fun setPresentationTime(nsecs: Long) {
        eglCore.setPresentationTime(eglSurface, nsecs)
    }
}
