# Limelight diagnostics and loop timing

Select **Limelight diagnostics** in the **Tests** TeleOp group. It looks up only
`Limelight3A` named `limelight`; it never initializes the robot, motors, or servos.
Diagnostics run during INIT and PLAY. STOP shuts down camera polling. If the device
is absent from the active configuration, fix the configuration and re-INIT.

## Controls

- A: switch SDK polling between 100 Hz (the robot default) and 20 Hz.
- B: pause/resume polling.
- X: enable/disable the once-per-second status HTTP request.
- D-pad left/right: request pipeline 0 through 9, one step per press. The initial
  pipeline is the current `Limelight.pipeline` setting, clamped to that range.

These switches affect this diagnostic session only. Pipeline selection changes the
camera's active pipeline; normal robot initialization requests its own configured pipeline.

## Reading the telemetry

`Configured` means the hardware map contains the named device. `Connected` is the
SDK's recent-communication check, not a physical USB cable detector. Check that
`Last SDK update` stays low and the camera timestamp keeps changing. Polling can
receive repeated copies of a frozen camera frame, so also inspect `Last frame
change` and `Observed new frames/sec`. The observed rate is sampled by this OpMode,
so it is limited by both polling and OpMode frequency; it is not camera FPS.

`Target valid` is from the cached result and may remain true after disconnection.
Use it together with `Fresh result`, connection, and polling state. Freshness uses
the robot's `Limelight.maximumFrameAgeMs`, combining staleness, time since the
camera timestamp changed, and capture/targeting latency. Connection without a
valid target can simply mean the pipeline sees no suitable target.

Check requested versus returned pipeline, tag IDs, and tx/ty in degrees. A working
camera can still produce no robot aiming correction: the robot filters IDs by
cell (redFar 30-33, redAudience 34-37, blueAudience 38-41, blueFar 42-45), requires
the configured pipeline and fresh frames, and applies its turret vision settings.
Robot correction now accepts one or more selected-goal tags with valid 3D depth
and compensates their offsets from the goal center; see `limelight-aiming.md`.
This helper displays raw detections without the cell filter.

Status CPU, temperature, camera FPS and pipeline type are snapshots requested once
per second while connected. The SDK may return a default status object when a
request fails; zero/empty status fields are not evidence of a healthy camera.
`Last status HTTP` measures that request. It is synchronous and can cause a spike
in this diagnostic's loop period. Toggle X off to compare. Telemetry is limited
to 10 Hz and its own cost is reported separately. Pipeline and polling controls
also perform SDK operations, so ignore their transition spikes when comparing.

## Why the full robot loop may be slower

Findings from the current source and the installed FTC SDK 12.0.0 sources:

1. `DebugNew.onUpdate()` reads transfer current every loop for overload rumble.
   `Intake` shares that sample with its cutoff logic, so this is one current read,
   not two. SDK `LynxDcMotorController.getMotorCurrent()` uses an ADC `sendReceive()`
   request; it is not supplied by the manual bulk cache. It runs even while idle.
2. `RobotNew.ShooterNew.periodic()` reads battery voltage every active loop when
   `volComp` is enabled. SDK `LynxVoltageSensor.getVoltage()` also performs an ADC
   `sendReceive()`. Debug telemetry adds another voltage read roughly every 100 ms.
3. `Drive.periodic()` calls Pedro's follower update, which refreshes Pinpoint over
   I2C and applies drive outputs. Bulk caching does not remove that I2C transaction.
4. The two turret servos have a 0.0001 command caching tolerance. Small heading or
   vision changes can therefore trigger frequent writes. This is a candidate to
   measure, not evidence that the turret is currently causing the slowdown.
5. Limelight polls `/results` and parses JSON on its SDK executor at 100 Hz.
   `getLatestResult()` itself reads cached data. Background polling may increase
   CPU, allocation/GC, and network work; compare 100/20 Hz and paused polling.
   Production `Limelight.periodic()` does not call `getStatus()`.
6. Debug telemetry goes to both Driver Station and Dashboard every 100 ms.
   Formatting/transmission can cause periodic spikes, though it is already throttled.

These are possible contributors, not a measured explanation of a regression. No
before/after robot timing or Control Hub logs were available. The current loop
updates drive/localization once, not twice, and contains no deliberate sleep.

`Alimzhan debug` now reports previous completed-loop work and separate
**drive/localization**, **controls + telemetry**, and **mechanisms** durations,
with session peaks so occasional spikes remain visible. Controls includes the
transfer-current read; mechanisms includes intake, shooter, Limelight, and turret.
The overall work duration also includes button reads and clearing bulk caches.
Peaks reset at START. Stage peaks can come from different iterations and must not
be added together. `Loop period` includes time between callbacks (SDK scheduling,
other threads and runtime pauses); `Loop work` measures work inside this callback.
The original `dt` remains in seconds: 0.020 means 20 ms, or about 50 Hz.

Compare the camera-only helper with the full robot and identify which stage is
large before changing sampling rates or protection behavior. Code compilation and
host tests cannot establish physical connection, detection quality, or Hub timing.
