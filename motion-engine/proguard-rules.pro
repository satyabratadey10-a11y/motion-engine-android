# Keep JNI native entry points
-keepclassmembers class com.tracker.motionengine.MotionEngine {
    native <methods>;
}

-keep class com.tracker.motionengine.MotionEngine$** { *; }
-keep class com.tracker.motionengine.** { *; }
