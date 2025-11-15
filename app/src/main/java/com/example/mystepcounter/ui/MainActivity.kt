package com.example.mystepcounter

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import kotlin.math.sqrt




fun f3(value: Float): String =
    String.format(java.util.Locale.US, "%.3f", value)

class MainActivity : AppCompatActivity() {
    private val vm: StepViewModel by viewModels()
    private val repo by lazy { StepRepository.get(application) }

    private fun hasActivityPermission(): Boolean =
        if (Build.VERSION.SDK_INT >= 29)
            ActivityCompat.checkSelfPermission(this, Manifest.permission.ACTIVITY_RECOGNITION) == PackageManager.PERMISSION_GRANTED
        else true

    private val requestActivityRecognition =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted || hasActivityPermission()) {
                // Let the service upgrade its FGS type and continue with location
                ContextCompat.startForegroundService(
                    this,
                    Intent(this, StepService::class.java).setAction(StepService.ACTION_PERMS_UPDATED)
                )
                ensureLocationPermission()
            } else {
                // Optional: explain why HW step sensors may not work without this
                android.widget.Toast.makeText(this, "Physical Activity permission denied", android.widget.Toast.LENGTH_SHORT).show()
                // Still continue to ask for location if you want GPS features:
                ensureLocationPermission()
            }
        }

    private fun ensureActivityPermission() {
        if (Build.VERSION.SDK_INT >= 29 && !hasActivityPermission()) {
            requestActivityRecognition.launch(Manifest.permission.ACTIVITY_RECOGNITION)
        } else {
            // Already granted or not needed on this API → move on to location
            ensureLocationPermission()
        }
    }

    private fun hasLocationPermission(): Boolean =
        ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
                ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED

    private val requestPostNotifications =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) {
            // After user answers notifications → ask Physical Activity next
            ensureActivityPermission()
        }

    private val requestLocation =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { grants ->
            val granted = grants[Manifest.permission.ACCESS_FINE_LOCATION] == true ||
                    grants[Manifest.permission.ACCESS_COARSE_LOCATION] == true ||
                    hasLocationPermission()
            if (granted) {
                // Tell the service to upgrade its FGS type + start GPS now
                ContextCompat.startForegroundService(
                    this,
                    Intent(this, StepService::class.java).setAction(StepService.ACTION_PERMS_UPDATED)
                )
            }
        }

    private fun ensureLocationPermission() {
        if (!hasLocationPermission()) {
            requestLocation.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            )
        }
    }

    /** Start service with an explicit user-initiated action so FGS promotion is allowed. */
    private fun startServiceSafely() {
        ContextCompat.startForegroundService(
            this,
            Intent(this, StepService::class.java).setAction(StepService.ACTION_START_FGS)
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        startServiceSafely()

        // POST_NOTIFICATIONS → ACTIVITY_RECOGNITION → LOCATION
        if (Build.VERSION.SDK_INT >= 33) {
            requestPostNotifications.launch(Manifest.permission.POST_NOTIFICATIONS)
        } else {
            ensureActivityPermission()
        }

        // Observe LiveData (lifecycle-aware)
        vm.kmhLD.observe(this) { speed ->
            findViewById<TextView>(R.id.KmH).text =
                "Current Km/H: ${if (speed != null) f3(speed * 3.6f) else "0.000"}"
        }
        vm.steps.observe(this) { steps ->
            findViewById<TextView>(R.id.count).text = "Steps: $steps"
        }
        vm.mode.observe(this) { mode ->
            findViewById<TextView>(R.id.modeText).text = "current mode: $mode"
        }
        vm.stepsToday.observe(this) { n ->
            findViewById<TextView>(R.id.todayStepsVal).text = n.toString()
        }
        vm.sensorsData.observe(this) {
            findViewById<TextView>(R.id.ax).text = "ax: ${f3(it.ax)}"
            findViewById<TextView>(R.id.ay).text = "ay: ${f3(it.ay)}"
            findViewById<TextView>(R.id.az).text = "az: ${f3(it.az)}"
            findViewById<TextView>(R.id.amag).text =
                "|a|: ${f3(sqrt(it.ax * it.ax + it.ay * it.ay + it.az * it.az))}"

            findViewById<TextView>(R.id.wx).text = "wx: ${f3(it.wx)}"
            findViewById<TextView>(R.id.wy).text = "wy: ${f3(it.wy)}"
            findViewById<TextView>(R.id.wz).text = "wz: ${f3(it.wz)}"
            findViewById<TextView>(R.id.wmag).text =
                "|ω|: ${f3(sqrt(it.wx * it.wx + it.wy * it.wy + it.wz * it.wz))} rad/s"

            findViewById<TextView>(R.id.lx).text = "lx: ${f3(it.ax - it.gx)}"
            findViewById<TextView>(R.id.ly).text = "ly: ${f3(it.ay - it.gy)}"
            findViewById<TextView>(R.id.lz).text = "lz: ${f3(it.az - it.gz)}"
            findViewById<TextView>(R.id.lmag).text =
                "|lin|: ${f3(sqrt((it.ax - it.gx) * (it.ax - it.gx) +
                        (it.ay - it.gy) * (it.ay - it.gy) +
                        (it.az - it.gz) * (it.az - it.gz)))}"

            findViewById<TextView>(R.id.gx).text = "gx: ${f3(it.gx)}"
            findViewById<TextView>(R.id.gy).text = "gy: ${f3(it.gy)}"
            findViewById<TextView>(R.id.gz).text = "gz: ${f3(it.gz)}"
            findViewById<TextView>(R.id.gmag).text =
                "|g|: ${f3(sqrt(it.gx * it.gx + it.gy * it.gy + it.gz * it.gz))}"

            findViewById<TextView>(R.id.hzText).text = "emaHz: ${f3(it.emaHz.toFloat())}"
        }

        findViewById<android.widget.Button>(R.id.queryDurationBtn).setOnClickListener {
            val etDurationMinutesInput = findViewById<android.widget.EditText>(R.id.durMinutesInput)
            val tvLastDurStepsVal = findViewById<TextView>(R.id.lastDurStepsVal)
            val tvAvgCadenceVal = findViewById<TextView>(R.id.avgCadenceVal)
            val minutes = etDurationMinutesInput.text.toString().trim().toIntOrNull()

            if (minutes == null || minutes <= 0) {
                tvLastDurStepsVal.text = "0"
                tvAvgCadenceVal.text = "0"
                android.widget.Toast.makeText(this, "Enter minutes > 0", android.widget.Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }

            val numOfStepsInLastMinutes = repo.stepsInLastMinutes(minutes)
            tvLastDurStepsVal.text = numOfStepsInLastMinutes.toString()

            val avgStepsPerDuration = repo.computeAvgStepsPerDuration(minutes.toLong())
            tvAvgCadenceVal.text = avgStepsPerDuration.toString()
        }
    }
}
