package com.example.mystepcounter

import android.Manifest
import android.annotation.SuppressLint
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.hardware.Sensor
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.os.Looper
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import androidx.core.app.ServiceCompat
import com.google.android.gms.location.*
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.collectLatest




class StepService : Service(), SensorEventListener {

    // Rolling window of accurate fixes for fallback speed
    private val recentFixes = ArrayDeque<Location>()   // keep ~6 fixes / ~10 s
    private var emaSpeedMps: Float? = null             // smoothed m/s

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    private lateinit var stepper: StepCounterZC

    private lateinit var fused: FusedLocationProviderClient
    private lateinit var locCallback: LocationCallback

    private lateinit var sm: SensorManager
    private var acc: Sensor? = null
    private var grav: Sensor? = null
    private var gyro: Sensor? = null

    private var emaHz = 0.0
    private var lastTsNs: Long? = null

    override fun onCreate() {
        super.onCreate()

        createNotifChannel()

        // Stepper
        stepper = StepCounterZC.getInstance(this,true)
        scope.launch { stepper.stepsFlow.collectLatest { StepBus.steps.value = it } }
        scope.launch { stepper.mode.collectLatest { StepBus.mode.value = it } }
        stepper.start()

        // Sensors (snapshot for UI)
        sm = getSystemService(SENSOR_SERVICE) as SensorManager
        acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        grav = sm.getDefaultSensor(Sensor.TYPE_GRAVITY)
        gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val rateUs = (1_000_000 / 60)
        acc?.let { sm.registerListener(this, it, rateUs, 0) }
        grav?.let { sm.registerListener(this, it, rateUs, 0) }
        gyro?.let { sm.registerListener(this, it, rateUs, 0) }

        // GPS → feeds speed to stepper for DRIVING/CYCLING
        fused = LocationServices.getFusedLocationProviderClient(this)
        locCallback = object : LocationCallback() {
            override fun onLocationResult(res: LocationResult) {
                val loc = res.lastLocation ?: return

                // 1) Provider speed if available (m/s)
                val providerMps: Float? = if (loc.hasSpeed()) loc.speed else null

                // 2) Keep short window of accurate fixes for fallback (≤20 m, ≤10 s, ≤6 points)
                if (loc.hasAccuracy() && loc.accuracy <= 20f) {
                    recentFixes.add(loc)
                    val now = loc.elapsedRealtimeNanos
                    while (recentFixes.size > 6 ||
                        (now - recentFixes.first().elapsedRealtimeNanos) > 10_000_000_000L) {
                        recentFixes.removeFirst()
                    }
                }

                // 3) Windowed fallback: total distance / total time
                var fallbackMps: Float? = null
                if (providerMps == null && recentFixes.size >= 2) {
                    var dist = 0f
                    var first = recentFixes.first()
                    var last = first
                    for (i in 1 until recentFixes.size) {
                        val a = recentFixes[i - 1]
                        val b = recentFixes[i]
                        val seg = a.distanceTo(b)
                        if (seg >= 3f) dist += seg
                        last = b
                    }
                    val dt = (last.elapsedRealtimeNanos - first.elapsedRealtimeNanos) / 1e9
                    if (dt >= 2.0 && dist >= 10f) fallbackMps = (dist / dt).toFloat()
                }

                // 4) Smooth (fast when provider gives speed, gentler for fallback)
                val rawMps = providerMps ?: fallbackMps
                if (rawMps != null) {
                    val alpha = if (providerMps != null) 0.6f else 0.35f
                    emaSpeedMps = if (emaSpeedMps == null) rawMps
                    else (1f - alpha) * emaSpeedMps!! + alpha * rawMps
                }

                // 5) Publish m/s (UI multiplies by 3.6 for km/h)
                val outMps = emaSpeedMps ?: providerMps ?: fallbackMps
                StepBus.speedMps.value = outMps
                stepper.updateSpeedMps(outMps)

                updateNotificationForMode()
            }
        }
        // NOTE: do NOT call startForeground() or startLocationUpdates() here.
    }

    /** Promote to foreground only when allowed (user-initiated actions). */
    private fun promoteToForeground() {
        val types =
            if (hasLocPerm()) {
                ServiceInfo.FOREGROUND_SERVICE_TYPE_HEALTH or
                        ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC or
                        ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION
            } else {
                ServiceInfo.FOREGROUND_SERVICE_TYPE_HEALTH or
                        ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC
            }

        try {
            ServiceCompat.startForeground(this, 42, buildNotif("Counting steps…"), types)
        } catch (e: android.app.ForegroundServiceStartNotAllowedException) {
            // Not allowed now (e.g., system/background restart) → degrade gracefully.
            (getSystemService(NOTIFICATION_SERVICE) as NotificationManager)
                .notify(43, buildNotif("Open app to resume step counter"))
            stopSelf()
        }
    }

    override fun onStartCommand(i: android.content.Intent?, flags: Int, startId: Int): Int {
        when (i?.action) {
            ACTION_START_FGS -> {
                promoteToForeground()
                startLocationUpdates()
                updateNotificationForMode()
                return START_STICKY
            }
            ACTION_PERMS_UPDATED -> {
                promoteToForeground()
                startLocationUpdates()
                updateNotificationForMode()
                return START_STICKY
            }
            else -> {
                // System restarted us in background with null/unknown intent.
                stopSelf()
                return START_NOT_STICKY
            }
        }
    }

    override fun onDestroy() {
        sm.unregisterListener(this)
        fused.removeLocationUpdates(locCallback)
        stepper.stop()
        scope.cancel()
        super.onDestroy()
    }

    // --- SensorEventListener (snapshot for UI) ---
    private var gx = 0f; private var gy = 0f; private var gz = 0f
    override fun onSensorChanged(e: android.hardware.SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_GRAVITY -> { gx = e.values[0]; gy = e.values[1]; gz = e.values[2] }
            Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_GYROSCOPE -> {
                val ax = if (e.sensor.type == Sensor.TYPE_ACCELEROMETER) e.values[0] else StepBus.sensors.value.ax
                val ay = if (e.sensor.type == Sensor.TYPE_ACCELEROMETER) e.values[1] else StepBus.sensors.value.ay
                val az = if (e.sensor.type == Sensor.TYPE_ACCELEROMETER) e.values[2] else StepBus.sensors.value.az

                val wx = if (e.sensor.type == Sensor.TYPE_GYROSCOPE) e.values[0] else StepBus.sensors.value.wx
                val wy = if (e.sensor.type == Sensor.TYPE_GYROSCOPE) e.values[1] else StepBus.sensors.value.wy
                val wz = if (e.sensor.type == Sensor.TYPE_GYROSCOPE) e.values[2] else StepBus.sensors.value.wz

                lastTsNs?.let { p ->
                    val dt = (e.timestamp - p) / 1e9
                    if (dt > 0) {
                        val inst = 1.0 / dt
                        val a = 0.10
                        emaHz = if (emaHz == 0.0) inst else (1 - a) * emaHz + a * inst
                    }
                }
                lastTsNs = e.timestamp

                StepBus.sensors.value = StepBus.sensors.value.copy(
                    ax = ax, ay = ay, az = az,
                    gx = gx, gy = gy, gz = gz,
                    wx = wx, wy = wy, wz = wz,
                    emaHz = emaHz
                )
            }
        }
    }
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) { /* no-op */ }

    // --- Location helpers ---
    private fun hasLocPerm() =
        ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
                ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED

    @SuppressLint("MissingPermission")
    private fun startLocationUpdates() {
        if (!hasLocPerm()) return
        val req = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L)
            .setMinUpdateIntervalMillis(1000L)
            .setMinUpdateDistanceMeters(0f)
            .build()
        fused.requestLocationUpdates(req, locCallback, Looper.getMainLooper())
    }

    // --- Foreground notification ---
    private fun createNotifChannel() {
        val id = "step_channel"
        val nm = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        if (nm.getNotificationChannel(id) == null) {
            nm.createNotificationChannel(
                NotificationChannel(id, "Step counting", NotificationManager.IMPORTANCE_LOW)
            )
        }
    }

    private fun buildNotif(text: String): Notification =
        NotificationCompat.Builder(this, "step_channel")
            .setSmallIcon(R.drawable.ic_launcher_foreground)
            .setContentTitle("Step Counter running")
            .setContentText(text)
            .setOngoing(true)
            .build()

    private fun updateNotificationForMode() {
        val n = buildNotif("Mode: ${StepBus.mode.value} • Steps: ${StepBus.steps.value}")
        (getSystemService(NOTIFICATION_SERVICE) as NotificationManager).notify(42, n)
    }

    override fun onBind(intent: android.content.Intent?) = null

    companion object {
        const val ACTION_START_FGS = "com.example.goforitGit.ACTION_START_FGS"
        const val ACTION_PERMS_UPDATED = "com.example.goforitGit.ACTION_PERMS_UPDATED"
    }
}
