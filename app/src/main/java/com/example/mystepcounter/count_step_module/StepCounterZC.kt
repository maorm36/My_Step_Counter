package com.example.mystepcounter

import android.content.Context
import android.content.SharedPreferences
import android.hardware.Sensor
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.util.Log
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt


/**
 * Orientation-robust step counter with persistence of recent history so public API
 * can return real values across app restarts.
 */
class StepCounterZC private constructor(
    private val context: Context,
    private var debugLogs: Boolean = true
) : SensorEventListener {

    // --- Tunables and getInstance() method ---
    companion object {
        // The getInstance() method of the singleton:
        // Volatile modifier is necessary due to two reasons:
        // 1) Reads and writes to the instance are not reordered by the compiler or processor.
        // 2) Changes made by one thread are visible to other threads immediately.
        @Volatile
        private var instance: StepCounterZC? = null
        fun getInstance(context: Context, debugLogs: Boolean = true): StepCounterZC {
            // we store the Application Context never the Activity/Service instance so the UI won't leak.
            val appCtx = context.applicationContext
            return instance ?: synchronized(this) {
                instance ?: StepCounterZC(appCtx, debugLogs).also { instance = it }
            }
        }

        // Tunables:
        // IIR LPF time constant (seconds) for gravity.
        private const val RC_GRAV = 0.80f

        // Minimum vertical-dominance ratio to consider a lobe “step-like”.
        private const val VDOM_MIN = 0.35f

        // Base floor and scale on σ for dynamic thresholds.
        private const val THR_FLOOR = 0.12f
        private const val K_SIGMA = 2.2f

        // Lower threshold as a fraction of high (hysteresis).
        private const val HYST_FRAC = 0.55f

        // Plausible step period bounds (~200 to 20 steps/min).
        private const val W_MIN = 0.30f
        private const val W_MAX = 3.00f

        // Allowed deviation from wEst when validating a step.
        private const val DELTA_MIN = 0.26f
        private const val DELTA_FRAC = 0.36f

        // EMA weight for updating cadence.
        private const val ALPHA_W = 0.22f

        // Persistence keys
        private const val K_STEP_TIMES = "stepTimesCsv"       // comma-separated longs
        private const val K_RECENT =
            "recentStepsCsv"         // semicolon list of time:period:ratio:amp
        private const val K_LAST_SAVE = "lastSaveMs"

        private const val TAG_DIAG = "STEPDIAG"
        private const val TAG_STEP = "STEP"
    }

    // --- Motion classification (non-intrusive) ---
    enum class MotionMode { UNKNOWN, STATIONARY, STANDING, WALKING, RUNNING, CYCLING, DRIVING }

    // Mode hysteresis
    private var pendingMode: MotionMode = MotionMode.UNKNOWN
    private var pendingSinceMs: Long = 0L

    private fun minDwellToEnter(m: MotionMode): Long = when (m) {
        MotionMode.RUNNING    -> 400L
        MotionMode.WALKING    -> 400L
        MotionMode.CYCLING    -> 800L
        MotionMode.DRIVING    -> 800L
        MotionMode.STANDING   -> 400L
        MotionMode.STATIONARY -> 600L
        else                  -> 300L
    }

    private val _mode = MutableStateFlow(MotionMode.UNKNOWN)
    val mode: StateFlow<MotionMode> = _mode.asStateFlow()

    // optional: feed GPS speed from outside (FusedLocation, etc.)
    @Volatile
    private var latestSpeedMps: Float? = null
    fun updateSpeedMps(speedMps: Float?) {
        latestSpeedMps = speedMps
    }

    // rolling window of recent samples for features (~8s)
    private data class WinSample(
        val tMs: Long,
        val vRatio: Float,
        val linRms: Float,
        val gyro: Float
    )

    private val win = ArrayDeque<WinSample>()
    private val WIN_MS = 8_000L

    // stash latest gyro magnitude so accel path can use it
    private var lastGyroMag = 0f
    private var lastGyroMs = 0L

    // --- Public observables ---
    private val _stepCount = MutableStateFlow(0)
    val stepCount: StateFlow<Int> = _stepCount

    // Optional hot stream alias for legacy collectors
    private val _stepsFlow = MutableStateFlow(0)
    val stepsFlow: StateFlow<Int> = _stepsFlow.asStateFlow()

    // Raw/derived samples for graphs or debugging
    data class Sample(
        // timestamp
        val tNanos: Long,
        // raw accel x,y,z
        val ax: Float, val ay: Float, val az: Float,
        // estimated gravity x,y,z
        val gx: Float, val gy: Float, val gz: Float,
        // vertical projection
        val v: Float,
        // linear acceleration magnitude
        val linMag: Float,
        // vertical-dominance ratio
        val vRatio: Float
    )

    // A hot stream of recent samples for graphs/diagnostics without retaining a large history in memory.
    private val _samples = MutableSharedFlow<Sample>(replay = 0, extraBufferCapacity = 256)
    val samples: SharedFlow<Sample> = _samples

    // --- Android sensors ---
    private val sm: SensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    private val stepDetector: Sensor? by lazy {
        sm.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR, /*wakeUp=*/true)
            ?: sm.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR)
    }
    private val stepSensor: Sensor? by lazy {
        sm.getDefaultSensor(Sensor.TYPE_STEP_COUNTER, /*wakeUp=*/true)
            ?: sm.getDefaultSensor(Sensor.TYPE_STEP_COUNTER)
    }
    @Volatile private var hwStepActive = false
    private var lastHwCount: Int? = null
    private var lastHwEventElapsedNs: Long? = null
    private var lastStoredStepMs: Long = 0L         // to keep stored timestamps strictly increasing

    private val accel: Sensor? = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT_WATCH) {
        sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER, /*wakeUp=*/false)
    } else {
        sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    }
    private val gyro: Sensor? =
        sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            ?: sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)

    // --- Gravity/filters ---
    private var gX = 0f
    private var gY = 0f
    private var gZ = 0f
    private var haveG = false

    private var lastTsNs: Long = 0L
    private var tSeconds: Float = 0f

    // Running stats for vertical channel
    private var vMean = 0f
    private var vM2 = 0f
    private var vCount = 0L

    // --- Step detection state machine ---
    // Two-phase detector: waiting in IDLE, then “armed” after a positive threshold crossing
    // until the symmetric negative crossing occurs (a stride pair).
    private enum class Phase { IDLE, ARMED_UP }

    private var phase = Phase.IDLE

    // Timestamp of previous negative crossing (stride period origin).
    private var lastDownS: Float = -1f

    // Exponential moving estimate of the stride period W (seconds).
    private var wEst: Float = 0f

    // Track local max/min between crossings to measure step amplitude.
    private var posPeak = 0f
    private var negPeak = 0f

    // --- Bookkeeping for external queries + persistence ---
    // What we remember per step to answer later API queries (cadence etc...)
    private data class StepFeature(
        val timeMs: Long,
        val periodS: Float,
        val vRatio: Float,
        val amp: Float
    )

    // Wall-clock timestamps (for “today”, last hour, etc.).
    private val stepTimesMs = ArrayDeque<Long>()

    // Small FIFO of last ~64 steps with features for analytics.
    private val recentSteps = ArrayDeque<StepFeature>()
    private val MAX_RECENT_STEPS = 64
    private val MAX_TIMESTAMPS = 10_000 // bounded window of timestamps

    // Persistence
    private val prefs: SharedPreferences =
        context.getSharedPreferences("stepzc_prefs", Context.MODE_PRIVATE)
    private var historyLoaded = false
    private var persistEvery = 0 // throttle writes

    // Cached thresholds
    private var thrHigh = THR_FLOOR
    private var thrLow = HYST_FRAC * THR_FLOOR

    // Diag
    private var lastSnapS = 0f


    // --------------------------------------------
    //                Lifecycle API
    // --------------------------------------------


    // setDebug(enabled) toggles logcat noise.
    fun setDebug(enabled: Boolean) {
        debugLogs = enabled
    }

    // reset() clears counters, phases, running stats, FIFOs, and persists the empty state.
    fun reset() {
        _stepCount.value = 0
        _stepsFlow.value = 0
        phase = Phase.IDLE
        lastDownS = -1f
        wEst = 0f
        posPeak = 0f
        negPeak = 0f
        vMean = 0f
        vM2 = 0f
        vCount = 0
        stepTimesMs.clear()
        recentSteps.clear()
        persistHistory() // keep disk tidy too
        if (debugLogs) Log.i(TAG_STEP, "Reset step counter + history")
    }

    // start() lazy-loads history once, registers the sensors with GAME (or UI) delay.
    fun start() {
        if (!historyLoaded) loadHistory()

        val rate = if (Build.VERSION.SDK_INT >= 21)
            SensorManager.SENSOR_DELAY_GAME else SensorManager.SENSOR_DELAY_UI

        stepDetector?.let { sm.registerListener(this, it, rate) }
            ?: Log.w(TAG_STEP, "step detector not available")

        stepSensor?.let { sm.registerListener(this, it, rate) }
            ?: Log.w(TAG_STEP, "step sensor not available")

        accel?.let { sm.registerListener(this, it, rate) }
            ?: Log.w(TAG_STEP, "Accelerometer not available")

        gyro?.let { sm.registerListener(this, it, rate) }
            ?: Log.w(TAG_STEP, "Gyroscope not available")

        if (debugLogs) Log.i(
            TAG_STEP,
            "Registered sensors: accel=${accel != null}, gyro=${gyro != null} (rate=$rate)"
        )
    }

    // stop() unregisters and persists history.
    fun stop() {
        sm.unregisterListener(this)
        persistHistory()
        if (debugLogs) Log.i(TAG_STEP, "Unregistered accelerometer listener + persisted history")
    }


    // --------------------------------------------
    //               Sensor callbacks
    // --------------------------------------------


    // No-op (accelerometer accuracy changes don’t impact our logic).
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // no-op
    }

    // Using signed vertical with two-sided hysteresis filters out single random spikes and insists on a full lobe pair (up then down) typical of gait.
    // Cadence gating plus an EMA of stride period makes the detector follow the user’s pace while resisting isolated outliers.
    // Amplitude and vertical-dominance requirements suppress hand-waving and pocket jostling that lack strong, gravity-aligned lobes.
    override fun onSensorChanged(event: android.hardware.SensorEvent) {

        when (event.sensor.type) {
            // hand tilting check
            Sensor.TYPE_GYROSCOPE -> {
                var wx = event.values[0];
                var wy = event.values[1];
                var wz = event.values[2]
                val wMag = sqrt((wx * wx + wy * wy + wz * wz).toDouble())
                if (wMag > 3.0)
                    return

                /** data for assessment of activity */
                lastGyroMag = wMag.toFloat()
                lastGyroMs = System.currentTimeMillis()
                return
            }

            // counting steps by a dedicated step counter
            Sensor.TYPE_STEP_COUNTER -> {
                hwStepActive = true

                val c = event.values[0].toInt()
                val tNs = event.timestamp // elapsedRealtimeNanos domain

                val prevC  = lastHwCount
                val prevNs = lastHwEventElapsedNs

                // Update baselines for next time
                lastHwCount = c
                lastHwEventElapsedNs = tNs

                // First event: just set the public total to the hardware value and exit
                if (prevC == null || prevNs == null) {
                    _stepCount.value = c
                    _stepsFlow.value = c
                    return
                }

                // Hardware counter can reset (e.g., sensor service restart / reboot)
                if (c < prevC) {
                    // Treat as new baseline; do not synthesize steps
                    return
                }

                val delta = c - prevC
                if (delta <= 0) return

                // Map sensor elapsed time to wall-clock time (ms)
                val nowWallMs = System.currentTimeMillis()
                val nowElapsedNs = android.os.SystemClock.elapsedRealtimeNanos()
                val tWallMs = nowWallMs - ((nowElapsedNs - tNs) / 1_000_000L)
                val prevWallMs = nowWallMs - ((nowElapsedNs - prevNs) / 1_000_000L)

                // Evenly distribute 'delta' steps across [prevWallMs .. tWallMs]
                var spanMs = tWallMs - prevWallMs
                if (spanMs < delta) spanMs = delta.toLong() // at least 1 ms per step to avoid duplicates

                var lastTs = lastStoredStepMs
                for (j in 1..delta) {
                    var ts = prevWallMs + (j * spanMs) / delta
                    // enforce strictly increasing timestamps (important for cadence)
                    if (ts <= lastTs) ts = lastTs + 1
                    stepTimesMs.add(ts)
                    lastTs = ts
                }
                lastStoredStepMs = lastTs

                while (stepTimesMs.size > MAX_TIMESTAMPS) stepTimesMs.removeFirst()

                // Publish counters
                val newTotal = _stepCount.value + delta
                _stepCount.value = newTotal
                _stepsFlow.value = newTotal

                if (++persistEvery >= 5) { persistEvery = 0; persistHistory() }
                return
            }

        }

        // beyond this point we are going to address only TYPE_ACCELEROMETER events
        if (event.sensor.type != Sensor.TYPE_ACCELEROMETER) return
        if (hwStepActive) return  // only skip ACC path after HW really started


        // Grab the timestamp and then on first call:
        // seed lastTsNs and initial gravity with the first reading, then return.
        val ts = event.timestamp
        if (lastTsNs == 0L) {
            lastTsNs = ts
            gX = event.values[0]; gY = event.values[1]; gZ = event.values[2]
            haveG = true
            return
        }

        // In short: compute a safe per-sample delta time in seconds,
        // accumulate it into a stable internal clock, and keep the last timestamp
        // so the next event’s dt is correct.
        // the variable ts, is the sensor’s event timestamp in nanoseconds since boot.
        // We take the difference to the previous event (lastTsNs) to get the elapsed time between samples.
        val dt = ((ts - lastTsNs).coerceAtLeast(1_000_000L)) / 1e9f
        lastTsNs = ts
        tSeconds += dt

        // getting Raw values of acceleration
        val ax = event.values[0];
        val ay = event.values[1];
        val az = event.values[2]

        // Gravity LPF (low-pass filter)
        val alpha = dt / (RC_GRAV + dt)
        gX += alpha * (ax - gX); gY += alpha * (ay - gY); gZ += alpha * (az - gZ)
        haveG = true

        // Linear accel calculation
        val lx = ax - gX;
        val ly = ay - gY;
        val lz = az - gZ
        val linMag = sqrt(lx * lx + ly * ly + lz * lz)

        // In this vertical projection part we are taking the 3-axis accelerometer vector
        // and extracting the component that points along the direction of the gravity's unit vector.
        val gMag = sqrt(gX * gX + gY * gY + gZ * gZ)
        if (gMag < 1e-4f) return
        val ux = gX / gMag;
        val uy = gY / gMag;
        val uz = gZ / gMag
        val v = lx * ux + ly * uy + lz * uz
        // This is a vertical-dominance metric: 1.0 if motion is along gravity, 0.0 if fully horizontal.
        // It lets us down-weight hand/pocket odd movements
        // and insist on “walk-like” vertical lobes to count steps.
        val vRatio = abs(v) / (linMag + 1e-6f)

        /** classify the current activity of the user which can be: driving, standing, etc... */
        val nowMs = System.currentTimeMillis()
        win.add(WinSample(nowMs, vRatio = vRatio, linRms = linMag, gyro = lastGyroMag))
        while (win.isNotEmpty() && (nowMs - win.first().tMs) > WIN_MS) win.removeFirst()
        classifyActivity(nowMs)

        // Welford for sigma(v)
        vCount += 1
        val dv = v - vMean
        vMean += dv / vCount
        vM2 += dv * (v - vMean)
        val sigmaV = if (vCount > 1) sqrt((vM2 / (vCount - 1)).toDouble()).toFloat() else 0f

        // Thresholds
        val dynamicThr = max(THR_FLOOR, K_SIGMA * sigmaV)
        thrHigh = min(dynamicThr, 1.6f)
        thrLow = HYST_FRAC * thrHigh

        // State machine
        when (phase) {
            Phase.IDLE -> {
                if (v > thrHigh && vRatio >= VDOM_MIN) {
                    phase = Phase.ARMED_UP
                    posPeak = v
                    negPeak = 0f
                    if (debugLogs) Log.i(
                        TAG_DIAG,
                        "[SC] arm(+), v=%.3f T=%.3f ratio=%.2f".format(v, thrHigh, vRatio)
                    )
                }
            }

            Phase.ARMED_UP -> {
                if (v > posPeak) posPeak = v
                if (v < negPeak) negPeak = v

                if (v < -thrLow) {
                    val tNow = tSeconds
                    var accepted = false
                    var reason = "?"

                    if (lastDownS > 0f) {
                        val W = tNow - lastDownS
                        if (W in W_MIN..W_MAX) {
                            wEst = if (wEst <= 0f) W else (1f - ALPHA_W) * wEst + ALPHA_W * W
                            val tol = max(DELTA_MIN, DELTA_FRAC * wEst)
                            if (abs(W - wEst) <= tol) {
                                val amp = posPeak - negPeak
                                val okAmp = amp >= (2.0f * thrHigh)
                                val okRatio = vRatio >= VDOM_MIN
                                if (okAmp && okRatio) {
                                    accepted = true
                                    reason = "ok"
                                } else reason = if (!okAmp) "amp" else "ratio"
                            } else reason = "cadence"
                        } else reason = "W"
                    } else {
                        wEst = 0f
                        reason = "prime"
                    }

                    val onFoot = when (_mode.value) {
                        MotionMode.WALKING, MotionMode.RUNNING -> true
                        MotionMode.UNKNOWN -> true
                        else -> false
                    }

                    if (accepted && onFoot) {
                        val Wcur = if (lastDownS > 0f) (tNow - lastDownS) else wEst
                        lastDownS = tNow
                        commitStep(tNow, Wcur, posPeak, negPeak, thrHigh, vRatio)
                    } else {
                        if (debugLogs) {
                            Log.w(
                                TAG_DIAG,
                                "[P2.V] reject: $reason pos=%.3f neg=%.3f W=%.3f wEst=%.3f thr=%.3f ratio=%.2f"
                                    .format(
                                        posPeak, negPeak,
                                        if (lastDownS > 0) tNow - lastDownS else -1f,
                                        wEst, thrHigh, vRatio
                                    )
                            )
                        }
                        lastDownS = tNow
                    }

                    phase = Phase.IDLE
                    posPeak = 0f
                    negPeak = 0f
                }
            }
        }

        // Emit sample
        _samples.tryEmit(
            Sample(
                tNanos = ts,
                ax = ax, ay = ay, az = az,
                gx = gX, gy = gY, gz = gZ,
                v = v,
                linMag = linMag,
                vRatio = vRatio
            )
        )

        // Diag snapshot
        if (debugLogs && (tSeconds - lastSnapS) >= 1.0f) {
            lastSnapS = tSeconds
            Log.i(
                TAG_DIAG,
                "[SNAP] t=%.3fs SC=%d V{T=%.3f/%.3f σ=%.3f} cadence{W=%.3f} ratio=%.2f"
                    .format(tSeconds, _stepCount.value, thrLow, thrHigh, sigmaV, wEst, vRatio)
            )
        }
    }

    // Responsibilities of this method are about handling valid steps events:
    // Increment the public step counter and notify observers.
    // Append this step to short-term history (timestamps & features), trimming old data.
    // Occasionally persist history to disk (to avoid frequent I/O).
    // Emit detailed debug logs.
    private fun commitStep(tNow: Float, W: Float, p: Float, n: Float, thr: Float, ratio: Float) {
        val sc = _stepCount.value + 1
        _stepCount.value = sc
        _stepsFlow.value = sc

        val nowMs = System.currentTimeMillis()

        // persist timestamps (bounded; prune >3 days)
        stepTimesMs.add(nowMs)
        while (stepTimesMs.size > MAX_TIMESTAMPS) stepTimesMs.removeFirst()
        val cutoff = nowMs - 3L * 24L * 60L * 60L * 1000L
        while (stepTimesMs.isNotEmpty() && stepTimesMs.first() < cutoff) stepTimesMs.removeFirst()

        // persist recent features
        recentSteps.add(
            StepFeature(
                timeMs = nowMs,
                periodS = if (W > 0f) W else wEst,
                vRatio = ratio,
                amp = (p - n)
            )
        )
        while (recentSteps.size > MAX_RECENT_STEPS) recentSteps.removeFirst()

        // Throttle disk writes (every ~5 steps)
        if (++persistEvery >= 5) {
            persistEvery = 0
            persistHistory()
        }

        if (debugLogs) {
            Log.i(
                TAG_DIAG, "[SC] +1 step ✓ W=%.3fs amp=%.3f thr=%.3f ratio=%.2f SC=%d"
                    .format(W, p - n, thr, ratio, sc)
            )
            Log.i(TAG_STEP, "Step detected at t=%.3fs (SC=$sc)".format(tNow))
        }
    }

    // classify the actual activity of the user while he uses the app
    private fun classifyActivity(nowMs: Long) {

        if (win.size < 20) return

        val n = win.size
        var sumVR = 0f;
        var sumL2 = 0f;
        var sumG2 = 0f

        for (s in win) {
            sumVR += s.vRatio; sumL2 += s.linRms * s.linRms; sumG2 += s.gyro * s.gyro
        }

        val vAvg = sumVR / n
        val linRms = kotlin.math.sqrt(sumL2 / n)
        val gyroRms = kotlin.math.sqrt(sumG2 / n)
        val speed = latestSpeedMps // latest speed in meters per second

        // steps/sec over the window you already use (~8s)
        val stepRate = countStepsSince(nowMs - WIN_MS).toFloat() / (WIN_MS / 1000f)

        // --- Raw condition flags (as you had) ---
        val drivingByGps = (speed ?: -1f) >= 6.94f // ≈ 25+ km/h
        val drivingBySensors =
            vAvg < 0.25f && stepRate < 0.2f && gyroRms < 0.35f && linRms in 0.10f..2.50f
        val cyclingByGps = speed != null && speed in 3.0f..6.8f // ≈ 10.8–24.5 km/h
        val cyclingBySensors = vAvg in 0.28f..0.55f && gyroRms in 0.60f..2.50f && stepRate < 0.5f

        // --- Step evidence (makes WALK/RUN "sticky") ---
        val steps4s = countStepsSince(nowMs - 4000L)
        val lastStepFresh = stepTimesMs.isNotEmpty() && (nowMs - stepTimesMs.last() <= 1500L)
        val hasStepEvidence = steps4s >= 3 || lastStepFresh

        // Treat gyro as optional; allow up to 5s since last gyro sample
        val gyroAvailable = (gyro != null)
        val freshGyro = !gyroAvailable || (nowMs - lastGyroMs) <= 5_000L

        // Stationary: essentially no motion (phone on table, etc.)
        val stillBySensors =
            stepRate < 0.02f &&
                    linRms < 0.06f &&
                    (!gyroAvailable || gyroRms < 0.06f) &&
                    freshGyro

        // Standing: tiny human sway (no steps), wider & more tolerant bands
        val standingBySensors =
            !hasStepEvidence &&
                    stepRate < 0.20f &&
                    linRms < 1f &&
                    (!gyroAvailable || gyroRms in 0.03f..1.00f) &&
                    freshGyro

        // --- Step-based sub-classification ---
        val runningBySteps =
            stepRate >= 2.2f && (wEst in 0.25f..0.45f) // ≈ 133–240 spm or fast period
        val walkingBySteps = stepRate >= 0.6f // ≈ 36+ spm (more robust than 0.4)

        // --- Compose a raw guess with "steps get priority" rule ---
        val rawGuess = when {
            hasStepEvidence && runningBySteps -> MotionMode.RUNNING
            hasStepEvidence && walkingBySteps -> MotionMode.WALKING
            drivingByGps || drivingBySensors  -> MotionMode.DRIVING
            cyclingByGps || cyclingBySensors  -> MotionMode.CYCLING
            standingBySensors                 -> MotionMode.STANDING
            stillBySensors                    -> MotionMode.STATIONARY
            else                              -> MotionMode.UNKNOWN
        }

        // --- Hysteresis / dwell + stickiness while stepping ---
        applyModeWithHysteresis(
            nowMs,
            rawGuess,
            hasStepEvidence,
            (drivingByGps || drivingBySensors || cyclingByGps || cyclingBySensors)
        )

        // Optional debug snapshot
        if (debugLogs) {
            Log.i(
                TAG_DIAG,
                "[MODE?] raw=$rawGuess vAvg=%.2f linRms=%.2f gyroRms=%.2f stepRate=%.2f speed=%s"
                    .format(vAvg, linRms, gyroRms, stepRate, speed?.toString() ?: "n/a")
            )
        }
    }

    /** Fast, transition-aware dwell.
     *  Returns 0 for "strong-evidence" transitions (commit immediately),
     *  small values for common quick flips (walk<->run, stop<->walk),
     *  and -1 to indicate "use fallback dwell".
     */
    private fun dwellForTransition(
        current: MotionMode,
        target: MotionMode,
        hasStepEvidence: Boolean,
        vehicleLikely: Boolean
    ): Long {
        val speedKmh = (latestSpeedMps ?: -1f) * 3.6f

        // Immediate on-foot if we have step evidence
        if ((target == MotionMode.WALKING || target == MotionMode.RUNNING) && hasStepEvidence) {
            return 0L
        }

        // Immediate vehicle if clearly fast and no steps now
        if ((target == MotionMode.DRIVING || target == MotionMode.CYCLING) &&
            !hasStepEvidence && (speedKmh >= 25f || (vehicleLikely && speedKmh >= 15f))
        ) {
            return 0L
        }

        // Very quick between WALKING <-> RUNNING
        if ((current == MotionMode.WALKING && target == MotionMode.RUNNING) ||
            (current == MotionMode.RUNNING && target == MotionMode.WALKING)
        ) {
            return 200L
        }

        // Quick from STATIONARY <-> WALKING
        if ((current == MotionMode.STATIONARY && target == MotionMode.WALKING) ||
            (current == MotionMode.WALKING && target == MotionMode.STATIONARY)
        ) {
            return 300L
        }

        // Quick around standing
        if ((current == MotionMode.STATIONARY && target == MotionMode.STANDING) ||
            (current == MotionMode.STANDING   && target == MotionMode.STATIONARY)) {
            return 200L
        }
        if ((current == MotionMode.STANDING && target == MotionMode.WALKING) ||
            (current == MotionMode.WALKING  && target == MotionMode.STANDING)) {
            return 250L
        }

        return -1L // signal: use fallback minDwellToEnter(target)
    }

    private fun applyModeWithHysteresis(
        nowMs: Long,
        rawGuess: MotionMode,
        hasStepEvidence: Boolean,
        vehicleLikely: Boolean
    ) {
        val current = _mode.value
        val speedKmh = (latestSpeedMps ?: 0f) * 3.6f

        // While currently on-foot, don't drop to vehicle/unknown if steps are still coming,
        // unless speed is clearly high (let vehicle preempt when undeniably moving fast).
        if ((current == MotionMode.WALKING || current == MotionMode.RUNNING) &&
            (rawGuess == MotionMode.CYCLING || rawGuess == MotionMode.DRIVING ||
                    rawGuess == MotionMode.UNKNOWN  || rawGuess == MotionMode.STATIONARY)
        ) {
            if (hasStepEvidence && speedKmh < 25f) return
        }

        // To enter a vehicle mode, require either: no step evidence or clear GPS speed.
        if ((rawGuess == MotionMode.DRIVING || rawGuess == MotionMode.CYCLING) &&
            hasStepEvidence && speedKmh < 10f
        ) {
            return
        }

        // Compute a fast/transition-aware dwell; fall back to per-mode dwell if needed.
        val fast = dwellForTransition(current, rawGuess, hasStepEvidence, vehicleLikely)
        val dwell = if (fast >= 0) fast else minDwellToEnter(rawGuess)

        // If this is a new candidate...
        if (rawGuess != pendingMode) {
            pendingMode = rawGuess
            pendingSinceMs = nowMs

            // ...and dwell is 0, commit immediately (true "real-time" flip).
            if (dwell == 0L && rawGuess != current) {
                _mode.value = rawGuess
                if (debugLogs) Log.i(TAG_DIAG, "[MODE] $rawGuess (instant)")
            }
            return
        }

        // Same candidate as last tick — check dwell time.
        if (rawGuess != current && (nowMs - pendingSinceMs) >= dwell) {
            _mode.value = rawGuess
            if (debugLogs) Log.i(
                TAG_DIAG,
                "[MODE] $rawGuess (confirmed after ${nowMs - pendingSinceMs} ms; dwell=$dwell)"
            )
        }
    }


    // ------------------------------------------------
    //             helper method count
    // ------------------------------------------------


    private fun countStepsSince(sinceMs: Long): Int {
        if (stepTimesMs.isEmpty()) return 0
        var c = 0
        for (t in stepTimesMs) if (t >= sinceMs) c++
        return c
    }


    // -----------------------------------------------------
    //         persistence layer (simple, bounded CSV)
    // -----------------------------------------------------


    private fun persistHistory() {
        val tsCsv = stepTimesMs.joinToString(",")
        val recCsv = recentSteps.joinToString(";") {
            "${it.timeMs}:${it.periodS}:${it.vRatio}:${it.amp}"
        }
        prefs.edit()
            .putString(K_STEP_TIMES, tsCsv)
            .putString(K_RECENT, recCsv)
            .putLong(K_LAST_SAVE, System.currentTimeMillis())
            .apply()
        if (debugLogs) Log.i(
            TAG_STEP,
            "Persisted history: steps=${stepTimesMs.size}, recent=${recentSteps.size}"
        )
    }

    private fun loadHistory() {
        historyLoaded = true
        val now = System.currentTimeMillis()
        // step timestamps
        prefs.getString(K_STEP_TIMES, null)?.let { csv ->
            if (csv.isNotBlank()) {
                csv.split(',').forEach { tok ->
                    tok.toLongOrNull()?.let { stepTimesMs.add(it) }
                }
            }
        }
        // prune anything older than 3 days
        val cutoff = now - 3L * 24L * 60L * 60L * 1000L
        while (stepTimesMs.isNotEmpty() && stepTimesMs.first() < cutoff) stepTimesMs.removeFirst()
        while (stepTimesMs.size > MAX_TIMESTAMPS) stepTimesMs.removeFirst()

        // recent features
        prefs.getString(K_RECENT, null)?.let { csv ->
            if (csv.isNotBlank()) {
                csv.split(';').forEach { item ->
                    val parts = item.split(':')
                    if (parts.size == 4) {
                        val t = parts[0].toLongOrNull()
                        val w = parts[1].toFloatOrNull()
                        val r = parts[2].toFloatOrNull()
                        val a = parts[3].toFloatOrNull()
                        if (t != null && w != null && r != null && a != null) {
                            recentSteps.add(StepFeature(t, w, r, a))
                        }
                    }
                }
            }
        }
        while (recentSteps.size > MAX_RECENT_STEPS) recentSteps.removeFirst()

        if (debugLogs) Log.i(
            TAG_STEP,
            "Loaded history: steps=${stepTimesMs.size}, recent=${recentSteps.size}"
        )
    }
}
