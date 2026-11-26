# Shooter encoder failover

`RobotNew.ShooterNew` and the older `Shooter` controller sample both encoders once
per periodic update. They start with `shooter1` (left). The selected sample supplies
velocity, acceleration, PID feedback, and readiness checks. Both encoders must use
the same ticks-per-revolution scale; this follows the existing shared shooter TPS
target and must be verified on the robot.

With a positive target and at least 0.2 commanded motor power, a selected encoder
below 100 TPS while the other reports at least 100 TPS is suspect. After 0.25 seconds
of continuous disagreement, feedback switches to the other encoder. For low targets,
the threshold is capped at 20% of the target. A non-finite selected reading switches
immediately if the backup is finite. The controller keeps its selection until that
encoder also fails; recovery of the original encoder alone does not switch it back.

Source changes reset PID history and suppress the acceleration jump. If both
readings stay below the threshold for 1.5 seconds of sustained commanded power,
the controller latches a fault, stops both motors, closes the stopper, and rejects
further feeding requests. Two non-finite readings fault immediately when enabled.
Velocity becomes NaN so readiness checks cannot pass. Explicitly turn the shooter
off and on, or restart the OpMode, to clear the fault. Repeated `turnOn()` calls
without `turnOff()` do not clear it.

Dashboard exposes the thresholds and delays under `ShooterEncoderFeedback`.
Defaults are initial detection settings, not hardware-validated calibration.
`Alimzhan debug` shows the source, fault state, and raw left/right TPS magnitudes.

This detects near-zero and non-finite feedback. It cannot reliably identify a
plausible frozen reading, incorrect encoder scale, arbitrary spikes, or distinguish
two failed encoders from a mechanically stalled shooter. Verify both raw readings
at low motor power before validating automatic failover on hardware.

Host tests cover startup, transient dropouts, switching in either direction,
non-finite feedback, fault latching/reset, acceleration continuity, and the
RobotNew controller's motor shutdown, stopper blocking, and readiness behavior.
