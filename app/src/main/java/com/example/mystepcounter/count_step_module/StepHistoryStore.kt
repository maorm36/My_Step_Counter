package com.example.mystepcounter

import android.content.Context



// helper for loading the lists of the timestamps of the steps and the recent steps that were recorded
class StepHistoryStore(private val ctx: Context) {
    private val prefs = ctx.getSharedPreferences("stepzc_prefs", Context.MODE_PRIVATE)

    fun loadStepTimestamps(): List<Long> =
        prefs.getString("stepTimesCsv", null)
            ?.takeIf { it.isNotBlank() }
            ?.split(',')
            ?.mapNotNull { it.toLongOrNull() } ?: emptyList()

    fun loadRecentSteps(): List<Float> =
        prefs.getString("recentStepsCsv", null)
            ?.takeIf { it.isNotBlank() }
            ?.split(';')
            ?.mapNotNull { it.split(':').getOrNull(1)?.toFloatOrNull() } ?: emptyList()
}
