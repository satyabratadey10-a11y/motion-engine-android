package com.tracker.smotion

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.RectF
import android.media.MediaMetadataRetriever
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.widget.SeekBar
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import androidx.media3.common.MediaItem
import androidx.media3.common.Player
import androidx.media3.exoplayer.ExoPlayer
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.tracker.smotion.databinding.ActivityMainBinding
import com.tracker.smotion.databinding.DialogExportProgressBinding
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import java.io.File
import java.util.Locale

/**
 * Main Activity for the SMotion application.
 * Provides video import, interactive touch-to-track bounding box selection,
 * real-time video playback preview, parameter tuning, and hardware MediaCodec export.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var exoPlayer: ExoPlayer? = null

    private var currentVideoUri: Uri? = null
    private var videoWidth = 0
    private var videoHeight = 0
    private var videoDurationMs = 0L

    private var currentBox = RectF()
    private var currentZoom = 1.20f
    private var currentSmooth = 0.40f
    private var drawReticle = true

    private var exportJob: Job? = null
    private var currentProcessor: SMotionVideoProcessor? = null
    private var progressDialog: AlertDialog? = null
    private var progressBinding: DialogExportProgressBinding? = null

    private val progressHandler = Handler(Looper.getMainLooper())
    private val progressRunnable = object : Runnable {
        override fun run() {
            exoPlayer?.let { player ->
                if (player.isPlaying) {
                    val pos = player.currentPosition
                    val dur = player.duration.coerceAtLeast(1L)
                    binding.videoSeekBar.progress = ((pos.toFloat() / dur.toFloat()) * 1000).toInt()
                    binding.tvVideoTime.text = "${formatTime(pos)} / ${formatTime(dur)}"
                }
            }
            progressHandler.postDelayed(this, 250)
        }
    }

    // Video Import Picker
    private val videoPickerLauncher = registerForActivityResult(
        ActivityResultContracts.GetContent()
    ) { uri: Uri? ->
        if (uri != null) {
            loadVideo(uri)
        }
    }

    // Permission Request Launcher
    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val granted = permissions.entries.all { it.value }
        if (granted) {
            videoPickerLauncher.launch("video/*")
        } else {
            Toast.makeText(this, "Storage permissions are required to import and export videos.", Toast.LENGTH_LONG).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        setupUI()
        setupExoPlayer()
        checkPermissionsAndPrompt()
    }

    private fun setupUI() {
        // Video Import buttons
        val importAction = {
            if (hasRequiredPermissions()) {
                videoPickerLauncher.launch("video/*")
            } else {
                requestRequiredPermissions()
            }
        }
        binding.btnImportTop.setOnClickListener { importAction() }
        binding.btnSelectVideo.setOnClickListener { importAction() }

        // Bounding Box Overlay Listener
        binding.boundingBoxOverlay.setOnBoxChangedListener { box ->
            currentBox = box
            binding.tvBoxCoordinates.text = String.format(
                Locale.US,
                "Box: [X: %d, Y: %d, W: %d, H: %d]",
                box.left.toInt(), box.top.toInt(), box.width().toInt(), box.height().toInt()
            )
        }

        // Preset Chips
        binding.btnPresetSmall.setOnClickListener {
            binding.boundingBoxOverlay.setPresetBox(0.10f, 0.10f)
        }
        binding.btnPresetMedium.setOnClickListener {
            binding.boundingBoxOverlay.setPresetBox(0.20f, 0.20f)
        }
        binding.btnPresetLarge.setOnClickListener {
            binding.boundingBoxOverlay.setPresetBox(0.35f, 0.35f)
        }

        // Parameter Tuning Sliders
        binding.sliderZoom.addOnChangeListener { _, value, _ ->
            currentZoom = value
            binding.tvZoomValue.text = String.format(Locale.US, "Zoom: %.2fx", value)
        }

        binding.sliderSmooth.addOnChangeListener { _, value, _ ->
            currentSmooth = value
            val label = when {
                value < 0.25f -> "Cinematic Glidecam"
                value < 0.60f -> "Balanced"
                else -> "Fast Action"
            }
            binding.tvSmoothValue.text = String.format(Locale.US, "Smoothness: %.2f (%s)", value, label)
        }

        binding.switchDrawReticle.setOnCheckedChangeListener { _, isChecked ->
            drawReticle = isChecked
        }

        // Play/Pause button
        binding.btnPlayPause.setOnClickListener {
            exoPlayer?.let { player ->
                if (player.isPlaying) {
                    player.pause()
                    binding.btnPlayPause.text = "▶"
                } else {
                    player.play()
                    binding.btnPlayPause.text = "❚❚"
                }
            }
        }

        // Seekbar
        binding.videoSeekBar.max = 1000
        binding.videoSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                if (fromUser) {
                    exoPlayer?.let { player ->
                        val target = (progress / 1000f * player.duration).toLong()
                        player.seekTo(target)
                    }
                }
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        // Export Button
        binding.btnExport.setOnClickListener {
            startExportPipeline()
        }
    }

    private fun setupExoPlayer() {
        exoPlayer = ExoPlayer.Builder(this).build().apply {
            repeatMode = Player.REPEAT_MODE_ALL
            addListener(object : Player.Listener {
                override fun onIsPlayingChanged(isPlaying: Boolean) {
                    binding.btnPlayPause.text = if (isPlaying) "❚❚" else "▶"
                }
            })
        }
        binding.playerView.player = exoPlayer
    }

    private fun loadVideo(uri: Uri) {
        currentVideoUri = uri
        exoPlayer?.apply {
            setMediaItem(MediaItem.fromUri(uri))
            prepare()
            play()
        }

        // Probe dimensions via MediaMetadataRetriever
        try {
            val retriever = MediaMetadataRetriever()
            retriever.setDataSource(this, uri)
            val wStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_WIDTH)
            val hStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_HEIGHT)
            val durStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)
            val rotStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_ROTATION) ?: "0"

            val rawW = wStr?.toIntOrNull() ?: 1280
            val rawH = hStr?.toIntOrNull() ?: 720
            val rot = rotStr.toIntOrNull() ?: 0

            videoWidth = if (rot == 90 || rot == 270) rawH else rawW
            videoHeight = if (rot == 90 || rot == 270) rawW else rawH
            videoDurationMs = durStr?.toLongOrNull() ?: 3000L

            retriever.release()

            binding.boundingBoxOverlay.setVideoDimensions(videoWidth, videoHeight)
            binding.emptyStateView.visibility = View.GONE
            binding.playerControlsBar.visibility = View.VISIBLE

            progressHandler.post(progressRunnable)

        } catch (e: Exception) {
            Toast.makeText(this, "Failed to read video dimensions: ${e.message}", Toast.LENGTH_SHORT).show()
        }
    }

    private fun startExportPipeline() {
        val uri = currentVideoUri
        if (uri == null) {
            Toast.makeText(this, "Please import a video first.", Toast.LENGTH_SHORT).show()
            return
        }

        // Pause preview playback
        exoPlayer?.pause()

        // Show progress dialog
        showProgressDialog()

        val processor = SMotionVideoProcessor(this)
        currentProcessor = processor

        exportJob = lifecycleScope.launch {
            try {
                val outputFile = processor.processVideo(
                    inputUri = uri,
                    initialBox = binding.boundingBoxOverlay.getSelectedBoxInVideoCoords(),
                    zoom = currentZoom,
                    smoothness = currentSmooth,
                    drawReticle = drawReticle
                ) { phase, pct, curr, total ->
                    runOnUiThread {
                        progressBinding?.let { pb ->
                            pb.tvProgressPhase.text = phase
                            pb.progressBar.progress = pct
                            pb.tvProgressPercent.text = "$pct%"
                            pb.tvProgressFrames.text = "$curr / $total frames"
                        }
                    }
                }

                dismissProgressDialog()
                showExportSuccessDialog(outputFile)

            } catch (e: InterruptedException) {
                dismissProgressDialog()
                Toast.makeText(this@MainActivity, "Export was cancelled.", Toast.LENGTH_SHORT).show()
            } catch (e: Exception) {
                dismissProgressDialog()
                MaterialAlertDialogBuilder(this@MainActivity)
                    .setTitle("Export Failed")
                    .setMessage(e.localizedMessage ?: e.toString())
                    .setPositiveButton("OK", null)
                    .show()
            }
        }
    }

    private fun showProgressDialog() {
        val dialogView = LayoutInflater.from(this).inflate(R.layout.dialog_export_progress, null)
        progressBinding = DialogExportProgressBinding.bind(dialogView)

        progressBinding?.btnCancelExport?.setOnClickListener {
            currentProcessor?.cancel()
            exportJob?.cancel()
            dismissProgressDialog()
        }

        progressDialog = MaterialAlertDialogBuilder(this)
            .setView(dialogView)
            .setCancelable(false)
            .create()

        progressDialog?.show()
    }

    private fun dismissProgressDialog() {
        progressDialog?.dismiss()
        progressDialog = null
        progressBinding = null
    }

    private fun showExportSuccessDialog(outputFile: File) {
        MaterialAlertDialogBuilder(this)
            .setTitle("⚡ Video Exported Successfully!")
            .setMessage("Saved to internal storage:\n${outputFile.absolutePath}")
            .setPositiveButton(R.string.play_output) { _, _ ->
                try {
                    startActivity(StorageHelper.getPlayIntent(this, outputFile))
                } catch (e: Exception) {
                    Toast.makeText(this, "No media player available to play video.", Toast.LENGTH_SHORT).show()
                }
            }
            .setNeutralButton(R.string.share_output) { _, _ ->
                try {
                    startActivity(StorageHelper.getShareIntent(this, outputFile))
                } catch (e: Exception) {
                    Toast.makeText(this, "Unable to share video.", Toast.LENGTH_SHORT).show()
                }
            }
            .setNegativeButton("Close", null)
            .show()
    }

    private fun formatTime(ms: Long): String {
        val totalSecs = (ms / 1000).toInt()
        val minutes = totalSecs / 60
        val seconds = totalSecs % 60
        return String.format(Locale.US, "%02d:%02d", minutes, seconds)
    }

    private fun hasRequiredPermissions(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            ContextCompat.checkSelfPermission(this, Manifest.permission.READ_MEDIA_VIDEO) == PackageManager.PERMISSION_GRANTED
        } else {
            ContextCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun requestRequiredPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            arrayOf(Manifest.permission.READ_MEDIA_VIDEO)
        } else {
            arrayOf(
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
            )
        }
        permissionLauncher.launch(permissions)
    }

    private fun checkPermissionsAndPrompt() {
        if (!hasRequiredPermissions()) {
            requestRequiredPermissions()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        progressHandler.removeCallbacks(progressRunnable)
        exoPlayer?.release()
        exoPlayer = null
    }
}
