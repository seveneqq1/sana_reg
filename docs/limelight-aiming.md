# Limelight turret correction

## Refined implementation request

Integrate the Limelight subsystem into Robot and RobotNew and all automatic
turret-aiming autonomous and TeleOp modes. The camera rotates with the turret
and is aligned horizontally with the shooter (user confirmed). Keep pose aiming
as the baseline and apply a small, bounded, rate-limited vision trim using only
fresh AprilTags belonging to the selected BIOBUZZ cell. Process each camera
frame once, reject stale/wrong-pipeline/nonfinite results, preserve turret limits,
manual control, intentional aim offsets, and existing shooting/path behavior.
Fall back to pose aiming when vision is unavailable. Stop polling on every
OpMode exit, including a stop before start. Expose telemetry and tuning settings,
and validate compilation and simulated control behavior without deploying.

## Implementation plan

1. Harden Limelight frame validation, freshness, target changes and lifecycle.
2. Test and implement a bounded vision trim shared by Turret automatic aiming.
3. Connect both Robot variants, automatic OpModes, stop cleanup and telemetry;
   run host tests, simulated hardware checks and the Android debug build.

Existing user edits in Turret, DebugNew and autonomous code must be preserved.
Fixed-position servo calibration OpModes must remain fixed-position tests.

## Sources and field mapping

- [FIRST BIOBUZZ Section 9, TU06, Figure 9-17](https://ftc-resources.firstinspires.org/ftc/game/manual-09)
  defines red far 30–33, red audience 34–37, blue audience 38–41, blue far 42–45.
- [Official Limelight FTC programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
  documents pipeline switching, polling, per-tag angular results and staleness.
- [Official visual servoing tutorial](https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing)
  describes proportional angular-error control and warns about excessive gain.
- Implementation is checked against the installed FTC Hardware 12.0.0 SDK sources.

This project's field axes place red at Y=58 and blue at Y=81. Rotating the
manual diagram into those axes gives west = far, east = audience:

| Existing target (inches) | Cell | Tags |
| --- | --- | --- |
| (53, 58) | redFar | 30–33 |
| (85, 58) | redAudience | 34–37 |
| (53, 81) | blueFar | 42–45 |
| (85, 81) | blueAudience | 38–41 |

Verify this inferred axis orientation against the real field during setup.

## Behavior and setup

Both Robot variants own one Limelight, initialize it after their hardware,
poll it before Turret, and stop it on shutdown. In scheduler-driven autonomous,
Limelight is constructed/registered before Turret; TeleOp uses Robot.periodic().
Turret.face() selects the cell from the intended field target. Tracking TeleOps
pass the original cell target separately from the driver's adjusted aim point.
Manual addYaw()/setYaw() suspend correction; automatic face() resumes it.
Existing nudge offsets and turn lead are included in the desired camera error.
Servo calibration OpModes intentionally retain their raw fixed-position commands.

Configure a Limelight3A named `limelight` in the robot configuration. A missing
device or absent target leaves normal pose aiming available. In the camera web
interface configure the selected pipeline (default 0) for AprilTags, family
36h11 and the manual's 3.25-inch tag size (0.08255 m). Correct camera orientation
so horizontal angular offsets describe left/right turret error. Full 3D is
needed for useful height estimates; horizontal correction uses per-tag angles
and does not use botPose or fixed robot-to-camera extrinsics.

Dashboard settings under `VisionAimTrim`:

| Setting | Default | Meaning |
| --- | --- | --- |
| enabled | true | Enables automatic vision trim |
| maxTrimDegrees | 8 | Maximum total correction from pose aiming |
| maxRateDegreesPerSecond | 6 | Maximum correction acquisition speed |
| gainPerSecond | 2 | Error response; unverified starting value |
| maxErrorDegrees | 15 | Rejects grossly misaligned observations |
| deadbandDegrees | 0.3 | Holds trim near the desired aim |
| minimumTags | 4 | Requires the full cell cluster to correct aim |
| aimOffsetDegrees | 0 | Additional desired aim offset, positive CCW |

`Limelight.maximumFrameAgeMs` defaults to 100 ms and includes capture/pipeline
latency, receipt staleness, and time since the camera timestamp last changed.
Frozen frames cannot keep correcting even if the camera continues returning
the same result over HTTP. `Limelight.pipeline` is selected during init; changing
it requires reinitializing the OpMode. Target changes invalidate old measurements.
Tipped detection now counts distinct images, rather than repeated polling of
the same image. The flag still latches until reset, as in MLP.

The confirmed physical mounting is 180 degrees roll and exactly 45 degrees
upward pitch, rotating with the turret. `Limelight.cameraPitch` defaults to 45
degrees and currently affects only height/tipped detection. `cameraHeight=10.986`
inches above the floor is user-reported, interpreted as the camera lens-center
height. `splitHeight=40` remains a legacy placeholder. These settings do not
enter the horizontal correction math.
Camera-relative correction is still `-tx` in degrees; physical roll/pitch
compensation and verification of angle signs after image orientation are pending.

### Mounting offsets in Dashboard

The following settings are exposed under `Limelight`. The user-reported camera
forward offset from the turret pivot is 5.101 inches. The corrected camera height
is 10.986 inches above the floor. The user clarified the turret pivot is 188 mm
above the floor (178 mm above the robot bottom), replacing the earlier 188.99 mm
report. With floor Z=0, `turretUpOffsetInches=188.0/25.4` (about 7.401574803
inches) and `cameraUpOffsetInches=3.584425197` (camera above the pivot). The camera
is confirmed directly in front of the pivot (`cameraLeftOffsetInches=0`),
and camera/shooter headings are aligned with turret forward (both yaw offsets
zero). The user now confirms the turret pivot is directly above the midpoint of
the four mecanum wheel centers, replacing the earlier report that it was behind
the robot origin. `turretForwardOffsetInches=0` and `turretLeftOffsetInches=0`
therefore assume this wheel-layout center is the localization reference. The user
believes the odometry pod offsets use that center; this reference has not been
physically verified. Shooter positional offsets remain unmeasured zero
placeholders. These new fields are stored
calibration settings only: they are not yet applied to aiming, distance estimates
or localization.

| Settings | Reference frame / units |
| --- | --- |
| `turretForwardOffsetInches`, `turretLeftOffsetInches`, `turretUpOffsetInches` | Robot pose origin to turret pivot; chassis forward/left/up, inches |
| `cameraForwardOffsetInches`, `cameraLeftOffsetInches`, `cameraUpOffsetInches` | Turret pivot to camera optical center; turret forward/left/up, inches |
| `shooterForwardOffsetInches`, `shooterLeftOffsetInches`, `shooterUpOffsetInches` | Turret pivot to a chosen point on the firing axis (for example, muzzle center); turret forward/left/up, inches |
| `cameraRollDegrees` | Physical optical-axis roll; defaults to 180 degrees |
| `cameraYawOffsetDegrees` | Camera heading relative to turret forward; positive CCW, defaults to 0 degrees |
| `shooterYawOffsetDegrees` | Firing-axis heading relative to turret forward; positive CCW, defaults to 0 degrees |

Turret forward means the turret's zero-yaw direction, rotating with the assembly;
turret left is 90 degrees CCW from that direction. Chassis forward/left are fixed
to the robot. Camera up offset is relative to the pivot, whereas `cameraHeight`
is absolute height above the floor. Physical roll describes the mount and must
not be reapplied blindly after a pipeline image rotation. Dashboard edits are
runtime settings; save measured values in source when calibration is complete.

## Verification and robot check

Host unit tests cover repeated frames, convergence for a biased turret, trim
and rate bounds, loss/gross-error fallback, disable/reset, invalid settings,
mechanical-limit windup and all four target mappings. A disposable Java harness
also exercises the actual Limelight and Turret classes against simulated SDK
hardware: frame validation, frozen timestamps, partial/wrong tags, servo output,
manual/nudge/heading-lead behavior, limits, tipping and lifecycle cleanup.

These checks cannot establish physical correction direction, camera calibration
or stability. Verify them with a stationary robot and the correct complete tag
cluster visible. Check `Limelight cell`, tags, error, frame age and `Vision trim`
in telemetry; a small aiming error should shrink as trim changes. Dashboard's
`VisionAimTrim.enabled=false` restores pose-only aiming.

Dropping below minimumTags, losing/staling the target, disabling correction or
changing cells clears the trim immediately. This can return up to 8 degrees
toward pose aiming; intermittent tag visibility can produce reacquisition motion.
Lowering minimumTags permits correction with a partial cluster but can shift
the apparent center toward the visible tags. This compensates observed aim error;
it does not diagnose slipping hardware, servo calibration or odometry drift.
