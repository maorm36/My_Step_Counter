package com.example.mystepcounter

import kotlinx.coroutines.flow.MutableStateFlow




// In Kotlin, objects allow you to define a class and create an instance of it in a single step.
// This is useful when you need either a reusable singleton instance or a one-time object.
object StepBus {

    val steps: MutableStateFlow<Int> = MutableStateFlow(0)
    val mode: MutableStateFlow<StepCounterZC.MotionMode> = MutableStateFlow(StepCounterZC.MotionMode.UNKNOWN)

    data class SensorSnapshot(
        val ax: Float, val ay: Float, val az: Float,
        val gx: Float, val gy: Float, val gz: Float,
        val wx: Float, val wy: Float, val wz: Float,
        val emaHz: Double
    )

    val sensors: MutableStateFlow<SensorSnapshot> =
        MutableStateFlow(SensorSnapshot(0f,0f,0f, 0f,0f,0f, 0f,0f,0f, 0.0))

    val speedMps: MutableStateFlow<Float?> = MutableStateFlow(null)

}

