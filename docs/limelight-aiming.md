# Limelight hybrid turret aiming

Ported from stilxso/kiroshi_reg main commit e1cdc32b6c00b72c14b2ca5a42e56eb464343a84 (2026-11-28). Robot wiring, servo calibration, pose targets, manual controls and shooter logic remain Regional_Q's.

Hardware name: `limelight`, FTC Limelight3A. The device is optional (`tryGet`). Configure AprilTags in camera pipeline 1; camera pipeline configuration itself is not supplied by the reference Java code.

`Turret.face()` polls the camera and averages raw horizontal angles of visible tags in the selected target cluster. Target coordinates select red audience (west/south, IDs 34–37), red (west/north, 30–33), blue audience (east/south, 38–41), or blue (east/north, 42–45), matching the reference hive coordinates. Camera results require validity and total frame age below 200 ms, including capture and targeting latency.

On a new frame, when turret tracking error is below 1.5 degrees and it has settled longer than the frame age, accumulate `offset += 0.15 * tx` outside a 0.75-degree deadband, clamped to +/-25 degrees. Hold the offset without tags. Settling uses the reference's 400 degrees/second estimate, capped at 0.1 seconds per periodic update; this is not measured servo feedback. The helper preserves the reference update rules.

Regional_Q yaw is bearing minus heading, opposite to the reference's heading minus bearing. Subtract the reference offset when applying it to Regional_Q yaw. Physical servo/camera correction direction still needs a robot check. No camera height, pitch, roll, translation or remembered mounting offsets are applied, matching the reference's raw-tx algorithm. No pose fusion, shooting gate or distance estimation is added.

Both Robot owners initialize and stop the camera. Auto cleanup stops it even before START. Alimzhan debug shows offset, tag visibility and frame age; D-pad up resets learned offset along with pose reset. Manual target locking excludes the learned correction to avoid applying it twice. New OpMode initialization resets the offset; switching clusters retains it as in the reference.

Validation: isolated JUnit checks cover new-frame and settling gates, hold-on-loss, reset, deadband, saturation and hive tag selection. Full compilation currently has four pre-existing errors in auto_new: missing RegShooter, selectWestFieldTarget and setTargetTPS. No robot deployment or physical validation has been performed.

## Review fixes (2026-11-28)

Servo mapping is centered at 0.5, retaining the current 330-degree full-scale calibration and the user-confirmed mechanical angle bounds 15–345 degrees (backward reference labeled 180). With the existing half-degree endpoint margins, effective travel is 15.5–344.5 degrees, or +/-164.5 degrees. Check the midpoint's physical backward alignment before use; full-scale calibration and midpoint still require hardware verification. The range calibration OpMode remains available. Out-of-range status is exposed in TeleOp telemetry.

Desired aim and vision target are now separate inputs to face(). The bearing difference, manual trim and heading lead are removed from the visual error so vision retains intentional aim bias. Settling is checked against the newly requested corrected angle, and unreachable requests cannot update the learned correction. TeleOp refreshes localization before user control/aim calculation and updates mechanisms afterward, once each loop.

Backward midpoint reference: hardware confirmation shows servo 0.5 points backward and increasing position turns counterclockwise. face() subtracts pi from the robot-relative field bearing; manual target reconstruction adds pi back. Reset remains (135, 9, 90 degrees), matching the reported robot front facing +Y. Full-scale gearing remains 395.102 degrees; the roughly 240-degree measured 0.2-to-0.8 sweep supports its direction and approximate scale.
