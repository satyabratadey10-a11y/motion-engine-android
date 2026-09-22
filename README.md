# MotionEngine Android SDK

A modular, high-performance Android NDK library compiling offline, non-AI, classical computer-vision video editing features into a universal `.aar` package.

[![Build Universal AAR](https://github.com/motionengine/motionengine-android/actions/workflows/build-aar.yml/badge.svg)](https://github.com/motionengine/motionengine-android/actions/workflows/build-aar.yml)

## Core Capabilities (Non-AI / Classical CV)

1. **Point Tracking (Sparse Lucas-Kanade Optical Flow)**
   - Multi-scale Gaussian/subsampling pyramid (Coarse-to-fine).
   - Bilinear subpixel interpolation and analytical spatial gradients ($I_x, I_y$).
   - Least-squares structural tensor inversion with eigenvalue condition thresholds.

2. **Object Tracking (ZNCC Template Matching Matrix)**
   - Zero-mean Normalized Cross-Correlation (ZNCC) invariant to linear illumination changes.
   - Fast 2-tier search (coarse stride search + fine subpixel parabolic peak refinement).
   - Real-time 3x3 tracking affine transformation matrix calculation.
   - Adaptive template update on high-confidence matches.

3. **Video Stabilization (Affine Motion Estimation + Moving Average Filter)**
   - Uniform feature grid detection across frame coordinates.
   - Optical flow feature vector matching with median distance outlier rejection.
   - Similarity affine transformation estimation ($dx, dy, d\theta, \text{scale}$).
   - Temporal moving-average circular buffer trajectory smoothing and centered 3x3 compensation matrix.

4. **Rotoscoping / Masking (Contour Deformation Tracking)**
   - Vertex-by-vertex deformation tracking using optical flow.
   - Global affine fallback estimation for textureless or occluded vertices.
   - Laplacian contour regularization to maintain smooth polygon boundaries.

---

## Architecture & File Structure

```
├── .github/
│   └── workflows/
│       └── build-aar.yml            # CI/CD GitHub Actions: JDK 17, NDK 25.2.9519653
├── gradle/
│   └── wrapper/
│       └── gradle-wrapper.properties
├── motion-engine/
│   ├── build.gradle.kts             # SDK 34, MinSDK 24, universal ABI filters, CMake C++17
│   ├── consumer-rules.pro           # Proguard keep rules for JNI native interfaces
│   ├── proguard-rules.pro
│   └── src/
│       └── main/
│           ├── AndroidManifest.xml  # Library manifest (com.tracker.motionengine)
│           ├── cpp/
│           │   ├── CMakeLists.txt   # CMake configuration linking log, android, jnigraphics
│           │   ├── motion-engine.h  # C++ tracking structures & CV engine declarations
│           │   └── motion-engine.cpp# Mathematical LK, ZNCC, affine estimation & JNI bridge
│           └── java/
│               └── com/tracker/motionengine/
│                   └── MotionEngine.kt # Kotlin high-level public API & companion loader
├── build.gradle.kts                 # Root project Gradle configuration
├── gradle.properties                # Build performance & AndroidX flags
├── gradlew                          # Gradle POSIX wrapper script
├── settings.gradle.kts              # Multi-module settings defining :motion-engine
└── README.md
```

---

## Universal Architecture Support

The native library `libmotionengine.so` is built for all major Android CPU architectures:
- `arm64-v8a` (64-bit ARM)
- `armeabi-v7a` (32-bit ARM)
- `x86_64` (64-bit x86 emulator / devices)
- `x86` (32-bit x86 emulator)

---

## Usage Guide (Kotlin)

### 1. Initialize MotionEngine
`MotionEngine` implements `AutoCloseable`, allowing seamless memory management:

```kotlin
import com.tracker.motionengine.MotionEngine
import java.nio.ByteBuffer

val engine = MotionEngine()
```

### 2. Sparse Optical Flow (Point Tracking)
```kotlin
val points = listOf(
    MotionEngine.Point(x = 120.0f, y = 240.0f),
    MotionEngine.Point(x = 340.0f, y = 480.0f)
)

val trackedPoints = engine.trackPoints(
    prevFrame = prevDirectByteBuffer,
    currFrame = currDirectByteBuffer,
    width = 1920,
    height = 1080,
    points = points,
    format = MotionEngine.FrameFormat.RGBA
)

for (pt in trackedPoints) {
    if (pt.isTracked) {
        println("Point tracked at: (${pt.x}, ${pt.y})")
    }
}
```

### 3. Object Bounding Box Tracking
```kotlin
// Step 1: Initialize template box on reference frame
val initialBox = MotionEngine.RectBox(x = 100f, y = 150f, width = 80f, height = 80f)
engine.initBoundingBox(
    frame = referenceBuffer,
    width = 1920,
    height = 1080,
    box = initialBox,
    format = MotionEngine.FrameFormat.RGBA
)

// Step 2: Track across subsequent frames
val (newBox, affineMatrix) = engine.trackBoundingBox(
    frame = nextFrameBuffer,
    width = 1920,
    height = 1080,
    format = MotionEngine.FrameFormat.RGBA
)

println("Tracked Box: ${newBox.x}, ${newBox.y} with confidence: ${newBox.confidence}")
```

### 4. Video Stabilization
```kotlin
val stabResult = engine.stabilizeFrame(
    frame = incomingFrameBuffer,
    width = 1920,
    height = 1080,
    format = MotionEngine.FrameFormat.RGBA
)

// Use 3x3 compensation matrix directly with OpenGL / Canvas / Vulkan
val glMatrix = stabResult.transformMatrix
```

### 5. Rotoscoping Polygon Mask Tracking
```kotlin
val initialMask = MotionEngine.PolygonMask(
    vertices = listOf(
        MotionEngine.Point(100f, 100f),
        MotionEngine.Point(200f, 100f),
        MotionEngine.Point(220f, 200f),
        MotionEngine.Point(120f, 220f)
    ),
    transformMatrix = FloatArray(9)
)

val warpedMask = engine.trackPolygonMask(
    prevFrame = prevBuffer,
    currFrame = currBuffer,
    width = 1920,
    height = 1080,
    mask = initialMask,
    format = MotionEngine.FrameFormat.RGBA
)
```

### 6. Resource Cleanup
```kotlin
engine.close()
```

---

## Building the Universal `.aar`

To build the release artifact locally:
```bash
./gradlew :motion-engine:assembleRelease
```

The output `.aar` package will be generated at:
```
motion-engine/build/outputs/aar/motion-engine-release.aar
```
