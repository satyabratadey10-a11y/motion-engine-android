package com.tracker.smotion

import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.media.MediaScannerConnection
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import androidx.core.content.FileProvider
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Storage and file utilities to ensure output videos are saved directly to
 * /Movies/SMotion/ and properly registered with the Android MediaStore.
 */
object StorageHelper {

    private const val SMOTION_SUBDIR = "SMotion"

    /**
     * Resolves and creates the /Movies/SMotion directory.
     */
    fun getSMotionDirectory(): File {
        val moviesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_MOVIES)
        val sMotionDir = File(moviesDir, SMOTION_SUBDIR)
        if (!sMotionDir.exists()) {
            sMotionDir.mkdirs()
        }
        return sMotionDir
    }

    /**
     * Creates a new output file in /Movies/SMotion/SMotion_yyyyMMdd_HHmmss.mp4
     */
    fun createOutputFile(): File {
        val dir = getSMotionDirectory()
        val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val filename = "SMotion_$timestamp.mp4"
        return File(dir, filename)
    }

    /**
     * Scans the file into the Android MediaStore so it appears instantly in Gallery apps.
     */
    fun scanMediaFile(context: Context, file: File, onComplete: ((Uri?) -> Unit)? = null) {
        MediaScannerConnection.scanFile(
            context,
            arrayOf(file.absolutePath),
            arrayOf("video/mp4")
        ) { _, uri ->
            onComplete?.invoke(uri)
        }
    }

    /**
     * Builds an ACTION_VIEW Intent to play the video in the default media player.
     */
    fun getPlayIntent(context: Context, file: File): Intent {
        val uri = FileProvider.getUriForFile(
            context,
            "${context.packageName}.fileprovider",
            file
        )
        return Intent(Intent.ACTION_VIEW).apply {
            setDataAndType(uri, "video/mp4")
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
    }

    /**
     * Builds an ACTION_SEND Intent to share the video with external applications.
     */
    fun getShareIntent(context: Context, file: File): Intent {
        val uri = FileProvider.getUriForFile(
            context,
            "${context.packageName}.fileprovider",
            file
        )
        return Intent(Intent.ACTION_SEND).apply {
            type = "video/mp4"
            putExtra(Intent.EXTRA_STREAM, uri)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
    }
}
