# Shooter distance tuning

Select **DebugNew Shooter Distance Tuning**. It reuses DebugNew's driving,
intake, feeding, aiming and shot compensation, with a separate Dashboard
`DebugNewShooterTuning.tuningTPS` field (initially 1234 TPS).

Both this OpMode and the regular DebugNew/Competition TeleOp start in **FIXED**
mode. Press **gamepad2 D-pad Up** once to toggle **FIXED / DISTANCE**. Holding
the button does not repeat the toggle. Gamepad1 D-pad Up still resets the pose
to `(134.8, 8.8, pi/2)`; only press it at that physical starting pose.

## Collect measurements

1. Verify the starting pose and odometry against measured field distances.
2. Set the single goal using Dashboard `DebugNew.targetX` and `targetY`.
   The current goal is `(110, 63)` inches in the existing field coordinate frame.
3. Stay in **FIXED** mode. Place the robot at a shooting distance, then adjust
   `DebugNewShooterTuning.tuningTPS` until shots work consistently. Y shoots,
   X intakes and A stops intake/feed, as in DebugNew.
4. Record **Distance to goal (in)** and the tuned **Shooter target TPS** at
   several distances. **Shooter TPS** shows measured speed / active controller
   target; the active target can include the existing temporary shot boost.
5. Send the `(distance in inches, target TPS)` pairs or the fitted equation.

Distance is the planar distance from the odometry robot pose to the fixed goal.
It does not include camera height, ball flight, or an unmeasured shooter offset.
Manual turret aim corrections do not move the goal used for speed calibration.

## Distance equations

The supplied fits are installed in `bobot/util/RegShooter.java`, with `x` in
inches and the result in TPS:

```text
Quadratic: TPS = 0.332561*x^2 - 30.533292*x + 1939.17245
Linear:    TPS = 11.02715*x + 722.1
```

`RegShooter.calibrated` starts true. **DISTANCE** mode uses the quadratic by
default; set Dashboard `RegShooter.useQuadratic` to false to use the linear fit.
Telemetry identifies the selected equation. The quadratic coefficients are
`quadraticA/B/C`; the linear coefficients are `slopeTPSPerInch` and `interceptTPS`.
Set `calibrated` false to temporarily fall back to the current fixed TPS.

Invalid distance or equation output commands zero TPS and cannot start feeding.
The measured distance range was not supplied, so no range clamp has been assumed.
These fits were supplied by the driver and have not been hardware-validated here.

The regular OpModes retain `RobotNew.ShooterNew.baseVel` as their fixed target.
Distance mode never overwrites it; toggling back restores that setting. The
runtime target override is cleared on stop and initialization.

## Autonomous

`auto_new`, `AutoNewFirstFlower` and `test_only_path_new` enable distance TPS at
the start of their routines. They use the same `RegShooter` equation selection
as TeleOp. The distance is read from the latest robot pose to
`Turret.selectWestFieldTarget(pose)`, matching their existing turret target.
Localization is updated before the command scheduler runs the mechanisms and
checks shooter readiness. Telemetry includes goal distance and requested/measured
TPS. PID, voltage compensation, shot boost and feeding timings are unchanged.

An explicit `setBaseVel` command switches to that fixed target. In particular,
`test_only_path_new` retains its existing 1850 TPS final shot. A later
`useDistanceTPS` command can re-enable the distance equation.

The original source snapshot is saved at
`output/shooter-backups/2026-11-27-before-distance-mode.zip`.
