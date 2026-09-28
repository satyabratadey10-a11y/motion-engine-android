# Keep MotionEngine JNI bindings and native methods
-keep class com.tracker.motionengine.** { *; }
-keepclassmembers class com.tracker.motionengine.** {
    native <methods>;
}

# Keep SMotion package
-keep class com.tracker.smotion.** { *; }
