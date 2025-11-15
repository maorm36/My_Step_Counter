package com.example.mystepcounter

import android.app.Application
import androidx.lifecycle.LiveData
import androidx.lifecycle.asLiveData
import androidx.lifecycle.liveData
import kotlin.math.roundToInt


class StepRepository private constructor(app: Application) {
    private val store = StepHistoryStore(app)

    // Live, lifecycle-aware streams for UI
    // The flow of data is from service -> bus -> repo
    val kmhLD: LiveData<Float?> = StepBus.speedMps.asLiveData()
    val stepsLD: LiveData<Int> = StepBus.steps.asLiveData()
    val modeLD: LiveData<StepCounterZC.MotionMode> = StepBus.mode.asLiveData()
    val sensorsLD: LiveData<StepBus.SensorSnapshot> = StepBus.sensors.asLiveData()

    // Derived values that don’t need constant recompute
    // Recompute only when someone is observing; trigger on step changes
    val stepsTodayLD: LiveData<Int> = liveData {
        StepBus.steps.collect {
            emit(stepsToday())
        }
    }


    // ------------------------------------------------------------
    //                 Public utility methods (API)
    // ------------------------------------------------------------


    private fun stepsToday(): Int {
        val times = store.loadStepTimestamps()
        val cal = java.util.Calendar.getInstance().apply {
            set(java.util.Calendar.HOUR_OF_DAY, 0); set(java.util.Calendar.MINUTE, 0)
            set(java.util.Calendar.SECOND, 0); set(java.util.Calendar.MILLISECOND, 0)
        }
        val start = cal.timeInMillis
        return times.count { it >= start }
    }

    // Returns average steps per minute over the last [timeMinutes] minutes.
    fun computeAvgStepsPerDuration(timeMinutes: Long): Int {
        // if the supplied timeMinutes are negative or equal to zero we exit with the value of 0
        if(timeMinutes <= 0)
            return 0

        // Guard & clamp
        val minutes = timeMinutes.coerceAtLeast(1L).coerceAtMost(24L * 60L)

        val windowMs = minutes * 60_000L
        val since = System.currentTimeMillis() - windowMs

        // Count steps in the window (timestamps are persisted in ms)
        val stepsInWindow = store.loadStepTimestamps().count { it >= since }

        // Average SPM = steps / minutes  (equivalently: steps * 60000 / windowMs)
        val spm = (stepsInWindow.toDouble() * 60_000.0) / windowMs.toDouble()

        // Round and clamp to sane human range
        return spm.roundToInt().coerceIn(0, 240)
    }

    /** Steps taken within the last [minutes] minutes (based on persisted timestamps). */
    fun stepsInLastMinutes(minutes: Int): Int {
        if (minutes <= 0) return 0
        val since = System.currentTimeMillis() - minutes * 60_000L
        val times = store.loadStepTimestamps()
        return times.count { it >= since }
    }

    companion object {
        @Volatile private var INSTANCE: StepRepository? = null
        fun get(app: Application): StepRepository =
            INSTANCE ?: synchronized(this) {
                INSTANCE ?: StepRepository(app).also { INSTANCE = it }
            }
    }
}