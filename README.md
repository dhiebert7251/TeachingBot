# Teaching-Bot (Java): AprilTag Vision

This branch builds on `teaching-bot-odometry-java` by adding one PhotonVision camera
doing AprilTag pose estimation, fusing its fixes into `DriveTrain`'s pose estimator to
correct the dead-reckoning drift the odometry branch's README named as its reason for
existing, and one new cross-subsystem command, `ApproachTagCommand`, built on top of
that. Every other design decision -- explicit-class commands, no lambdas, the same
physical robot -- carries over unchanged from `teaching-bot-poc-java`; see that
branch's README for the full reasoning behind those.

**Read this before anything else: this project has not been compiled or run.** See
[Verification status](#verification-status) below -- this branch has one additional,
more significant disclosed divergence from the real competition port than either
earlier Java branch did, and it's worth reading before trusting anything in
`Vision.java`.

## Contents

- [Verification status](#verification-status)
- [What changed from teaching-bot-odometry-java](#what-changed-from-teaching-bot-odometry-java)
- [Camera mounting](#camera-mounting)
- [Pose estimation: from odometry to a pose estimator](#pose-estimation-from-odometry-to-a-pose-estimator)
- [How Vision decides a measurement is trustworthy](#how-vision-decides-a-measurement-is-trustworthy)
- [ApproachTagCommand](#approachtagcommand)
- [Running tests](#running-tests)
- [Building and running this project](#building-and-running-this-project)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
- [Blank templates: subsystem and command](#blank-templates-subsystem-and-command)
- [Annotated source code](#annotated-source-code)
  - Core classes
    - [Constants.java](#srcmainjavafrcrobotconstantsjava)
    - [Main.java](#srcmainjavafrcrobotmainjava)
    - [Robot.java](#srcmainjavafrcrobotrobotjava)
    - [RobotContainer.java](#srcmainjavafrcrobotrobotcontainerjava)
    - [VisionMeasurement.java](#srcmainjavafrcrobotvisionmeasurementjava)
  - Subsystems
    - [DriveTrain.java](#srcmainjavafrcrobotsubsystemsdrivetrainjava)
    - [Elevator.java](#srcmainjavafrcrobotsubsystemselevatorjava)
    - [Gripper.java](#srcmainjavafrcrobotsubsystemsgripperjava)
    - [Shooter.java](#srcmainjavafrcrobotsubsystemsshooterjava)
    - [Trigger.java](#srcmainjavafrcrobotsubsystemstriggerjava)
    - [Vision.java](#srcmainjavafrcrobotsubsystemsvisionjava)
  - Commands
    - [ApproachTagCommand.java](#srcmainjavafrcrobotcommandsapproachtagcommandjava)
    - [DriveDistanceCommand.java](#srcmainjavafrcrobotcommandsdrivedistancecommandjava)
    - [EjectCommand.java](#srcmainjavafrcrobotcommandsejectcommandjava)
    - [FireCommand.java](#srcmainjavafrcrobotcommandsfirecommandjava)
    - [IntakeCommand.java](#srcmainjavafrcrobotcommandsintakecommandjava)
    - [LowerElevatorCommand.java](#srcmainjavafrcrobotcommandslowerelevatorcommandjava)
    - [RaiseElevatorCommand.java](#srcmainjavafrcrobotcommandsraiseelevatorcommandjava)
    - [ResetGyroCommand.java](#srcmainjavafrcrobotcommandsresetgyrocommandjava)
    - [SpinUpShooterCommand.java](#srcmainjavafrcrobotcommandsspinupshootercommandjava)
    - [TeleopDriveCommand.java](#srcmainjavafrcrobotcommandsteleopdrivecommandjava)
    - [TurnToAngleCommand.java](#srcmainjavafrcrobotcommandsturntoanglecommandjava)
  - Autonomous
    - [AutoChooser.java](#srcmainjavafrcrobotautonomousautochooserjava)
    - [AutoRoutines.java](#srcmainjavafrcrobotautonomousautoroutinesjava)
  - Tests
    - [ElevatorTest.java](#srctestjavafrcrobotelevatortestjava)
    - [RobotLifecycleTest.java](#srctestjavafrcrobotrobotlifecycletestjava)
    - [TriggerTest.java](#srctestjavafrcrobottriggertestjava)
    - [VisionTest.java](#srctestjavafrcrobotvisiontestjava)
    - [ApproachTagCommandTest.java](#srctestjavafrcrobotcommandsapproachtagcommandtestjava)
    - [DriveTrainTest.java](#srctestjavafrcrobotsubsystemsdrivetraintestjava)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Verification status

**Still not compiled, run, or tested** -- same sandboxed session, same network policy
blocking `frcmaven.wpi.edu` and the REVLib/CTRE/Studica/PhotonVision Maven hosts. See
`teaching-bot-poc-java`'s README for the full explanation; everything there applies
here too, including that branch's own `Robot.java` fix -- this branch inherited
`Robot.java` unchanged, so the same build-breaking bug (extending a nonexistent
`TimedCommandRobot` class instead of `TimedRobot`, with no `robotPeriodic()`
override to call the scheduler) was present here too, and has now been fixed here
the same way.

**This branch's one significant, deliberate divergence, disclosed plainly rather than
hidden:** `subsystems/Vision.java` constructs its `PhotonPoseEstimator` with a 2-arg
constructor (field layout + camera transform, no strategy argument) and calls
`estimateCoprocMultiTagPose(result)` directly, falling back to
`estimateLowestAmbiguityPose(result)`. The real competition port's own `Vision.java`
instead constructs `PhotonPoseEstimator` with an explicit 3rd argument,
`PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR`, and calls one `poseEstimator.update(result)`
per camera. Both patterns exist in PhotonLib for the 2026 season, per the vendor docs.
**This pattern has not been verified against the installed Java `v2026.3.2` PhotonLib
package** -- this file was read and reasoned about, never compiled or run, in this
session (see the top of this section). This file uses the 2-arg-constructor-plus-
multi-tag-estimate pattern anyway because it names the specific PhotonPoseEstimator
methods being exercised (`estimateCoprocMultiTagPose()` for the common multi-tag case,
falling back explicitly to `estimateLowestAmbiguityPose()`) rather than delegating that
choice to a `PoseStrategy` enum value whose internal fallback behavior is opaque from
the call site -- a rookie reading this file can see exactly which estimation method
ran without also reading PhotonLib's own source for what `MULTI_TAG_PNP_ON_COPROCESSOR`
does internally. That is a real, if modest, teaching-clarity reason to prefer it here,
but it is **not** evidence that this pattern is the one PhotonLib actually expects, or
that it compiles at all -- unlike the competition port's own pattern, which is real
production code even though it too was only read, never run, in this session. **If
`./gradlew build` fails inside `Vision.java`, this constructor call and the two
`estimate*Pose()` method names are the first place to check** -- see that file's own
class-level Javadoc for the same explanation in place, right next to the code it's
about.

Every other API used in this branch's new/changed files (`DifferentialDrivePoseEstimator`'s
constructor and `addVisionMeasurement()`/`getEstimatedPosition()`/`resetPosition()`,
`VecBuilder.fill()`, `Transform3d`/`Translation3d`/`Rotation3d`, `AprilTagFieldLayout`/
`AprilTagFields`) WAS cross-checked directly against the real competition port's own
`DriveTrain.java` and `Constants.java`, fetched and read in this session specifically
to confirm this branch's usage against them line by line -- these are not a
divergence, just the same "verify before trusting" discipline every earlier branch's
README already describes.

**Two smaller gaps found during a dedicated post-hoc code-review pass, worth naming
rather than smoothing over:**

- `DriveTrain.resetPose()` calls `resetEncoders()` before seeding the pose estimator
  with literal `0.0, 0.0` for left/right distance. That's self-consistent (it zeroes
  the distance, then tells the estimator "you're at zero distance from here"), and
  it's the exact same pattern this branch inherited unchanged from
  `teaching-bot-odometry-java`, where it was independently verified as logically
  sound. But it is NOT the same thing the real competition port's own `resetPose()`
  does -- that version never resets the hardware encoders, and instead passes the
  *live* `getLeftDistanceMeters()`/`getRightDistanceMeters()` into `resetPosition()`.
  The paragraph above, about this branch's pose-estimator API usage being "NOT a
  deliberate divergence" from the real robot's code, is accurate for the
  constructor/`update()`/`addVisionMeasurement()`/`getEstimatedPosition()` calls, but
  overstated as written for `resetPose()` specifically -- that one method's exact
  behavior is a real, if minor, difference between this teaching project and the
  competition robot, not a byte-for-byte match.
- `ApproachTagCommand`'s two bindings in `RobotContainer.java` (driver `a()`/`x()`)
  have no `.withTimeout(...)`, unlike `FireCommand`'s binding. If the command's
  internal PID chain never converges (an edge-of-field target, a stalled drivetrain,
  oscillation), the driver can't preempt it with the default teleop command and loses
  stick control until it resolves on its own. This is a real, undisclosed-until-now
  safety gap, worth fixing rather than leaving in place just because it mirrors the
  pattern `FireCommand`'s own binding already avoids. See
  [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
  below.

## What changed from teaching-bot-odometry-java

- **`Constants.java`** gained a `VisionConstants` nested class: camera name, the
  robot-to-camera mounting transform, vision quality-gating thresholds, the three
  standard-deviation matrices, and the two example `ApproachTagCommand` bindings'
  numbers (tag ID, standoff distances, turn offset).
- **New file, `VisionMeasurement.java`** -- a `record` bundling one vision pose fix
  (`estimatedPose`, `timestampSeconds`, `standardDeviations`, `numTagsUsed`). See that
  file's own Javadoc for why `record` is the right tool here.
- **New file, `subsystems/Vision.java`** -- the new subsystem. See
  [Pose estimation](#pose-estimation-from-odometry-to-a-pose-estimator) and
  [How Vision decides a measurement is trustworthy](#how-vision-decides-a-measurement-is-trustworthy)
  below.
- **`subsystems/DriveTrain.java`** -- swapped `DifferentialDriveOdometry` for
  `DifferentialDrivePoseEstimator`, added a `Vision` constructor parameter, and folds
  in a fresh vision measurement every loop in `periodic()`. `getPose()`/`resetPose()`
  keep their exact same names and signatures from the odometry branch (see
  [Verification status](#verification-status) above for one caveat on `resetPose()`'s
  exact behavior).
- **New file, `commands/ApproachTagCommand.java`** -- this project's one genuinely
  cross-subsystem command. See [ApproachTagCommand](#approachtagcommand) below.
- **`commands/DriveDistanceCommand.java`** -- fixed a real pose-corruption bug; see
  [Design decisions](#design-decisions-and-deliberate-simplifications) below.
- **`RobotContainer.java`** -- constructs `Vision` (before `DriveTrain`, which needs
  it), and binds `ApproachTagCommand` twice to the driver's A/X buttons.
- **New vendor dependency, `vendordeps/photonlib.json`** -- copied verbatim from the
  real competition port's own copy (`v2026.3.2`).
- **New tests:** `VisionTest.java`, `commands/ApproachTagCommandTest.java`, and one
  regression test added to `subsystems/DriveTrainTest.java` -- see
  [Running tests](#running-tests) below, including two testability wrinkles this
  branch specifically introduces.
- **`Robot.java`** -- carries the `TimedCommandRobot` -> `TimedRobot` fix described
  in [Verification status](#verification-status) above; otherwise unchanged.

## Camera mounting

`Constants.VisionConstants.ROBOT_TO_CAMERA` encodes the camera's physical mounting as
a `Transform3d` from the robot's center to the camera lens -- exactly the numbers
given for this robot: centered left/right, 3 inches back from the front of the frame,
1 foot above the floor, angled 15 degrees upward.

```java
public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
    new Translation3d((16.0 - 3.0) * 0.0254, 0.0, 1.0 * 0.3048),
    new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)
);
```

Two things worth explaining, since both are easy to get backwards:

- **"3 inches from the front of the frame" isn't the same number as "3 inches
  forward of center."** The frame is 32 inches front-to-back, so its center is 16
  inches back from the front edge. A camera 3 inches back from the front edge is
  therefore `16 - 3 = 13` inches forward of the robot's own center -- `Transform3d`,
  like everything else in `wpimath`, measures from the robot's origin (its center),
  not from any particular edge of the frame. Left/right offset is 0 because the
  camera is centered; height is a straight foot-to-meter conversion since
  `Translation3d`'s Z axis is already "up" from the floor.
- **Negative pitch tilts the camera up, not down.** `Rotation3d`'s pitch convention
  is easy to get backwards by guessing, and **this sign convention has not been
  independently verified in this session** -- it is asserted from `wpimath`'s own
  documented convention for `Rotation3d`'s pitch component (rotating a "straight
  ahead" translation by `Rotation3d(0, pitch, 0)` is documented to give a negative
  pitch a positive Z component, i.e. tilted up), not confirmed by actually running
  that rotation and inspecting the result, since this project has not been compiled
  or run at all (see [Verification status](#verification-status) above). Worth a
  rookie running that one-line experiment in Java once a build environment exists,
  rather than taking this comment on faith -- that's the whole point of writing it
  down as a claim someone can check, instead of a fact.

## Pose estimation: from odometry to a pose estimator

`DriveTrain` swapped its plain `DifferentialDriveOdometry` (the odometry branch) for a
`DifferentialDrivePoseEstimator`. The two take the same encoder/gyro inputs every
loop, but the pose estimator also accepts vision fixes via
`addVisionMeasurement(pose, timestampSeconds, stdDevs)`, and internally runs a Kalman
filter that blends a vision fix in proportionally to how confident it is (smaller
`stdDevs` = more trusted = pulled toward harder), rather than either ignoring vision
or snapping straight to it. Every loop, `DriveTrain.periodic()` asks `Vision` for its
best fresh measurement and, if there is one, feeds it in:

```java
Optional<VisionMeasurement> measurement = vision.getBestVisionMeasurementIfFresh();
measurement.ifPresent(m ->
    poseEstimator.addVisionMeasurement(m.estimatedPose(), m.timestampSeconds(), m.standardDeviations())
);
```

`getPose()` and `resetPose()` keep their exact same names and signatures from the
odometry branch -- a caller elsewhere in the codebase (like `ApproachTagCommand`)
doesn't need to know or care whether the pose behind them is pure dead reckoning or
vision-fused. This constructor/method pattern (`DifferentialDrivePoseEstimator(kinematics,
gyroAngle, leftDistance, rightDistance, initialPose)`, `getEstimatedPosition()`,
`resetPosition(...)`, `addVisionMeasurement(...)`) was cross-checked directly against
the real competition port's own `DriveTrain.java` -- with one caveat: see
[Verification status](#verification-status) above for `resetPose()`'s own specific,
smaller divergence (it resets the hardware encoders; the competition port's version
doesn't).

## How Vision decides a measurement is trustworthy

`subsystems/Vision.java`'s `computeMeasurement()` runs several checks before a camera
result becomes something `DriveTrain` is allowed to fuse in, each one guarding
against a specific, real failure mode rather than being defensive for its own sake:

- **No targets, no estimate.** `PhotonPoseEstimator` is asked for a multi-tag
  estimate first (`estimateCoprocMultiTagPose()`), falling back to a
  lowest-ambiguity single-tag estimate (`estimateLowestAmbiguityPose()`) when
  multi-tag PNP data isn't available -- see
  [Verification status](#verification-status) above for why this specific pair of
  method names is this branch's one flagged divergence from the real competition
  port's code.
- **A pose off the field is never real.** If the estimated X/Y falls outside the
  field's own dimensions (from the same `AprilTagFieldLayout` used to look up tag
  positions), it's thrown out -- a bad reprojection producing a robot "10 meters past
  the wall" is a sign of a bad estimate, not a real robot position.
- **A single ambiguous tag is untrustworthy.** One AprilTag alone is a genuinely
  ambiguous pose problem -- the same tag corners are consistent with two different
  camera poses, mirrored through the tag's plane. PhotonVision's own
  `getPoseAmbiguity()` score flags when that ambiguity is high; above
  `VisionConstants.MAX_AMBIGUITY`, the whole measurement is dropped rather than
  trusted.
- **Confidence (`stdDevs`) scales with how the estimate was made.** Multiple tags in
  view resolve that ambiguity geometrically, so `MULTI_TAG_STDDEVS` (tightest)
  applies whenever at least `VisionConstants.MIN_TAGS_FOR_MULTI_TAG` tags
  contributed. A single tag gets looser trust, and looser still
  (`SINGLE_TAG_FAR_STDDEVS` vs. `SINGLE_TAG_CLOSE_STDDEVS`) the farther away it is --
  distance amplifies any small pixel-level error in the corner detection into a
  larger real-world position error. A single tag farther than
  `VisionConstants.MAX_TAG_DISTANCE_METERS` is dropped outright rather than trusted
  at any confidence.
- **Staleness matters for both fusion and `isAnyVisionAvailable()`.**
  `getBestVisionMeasurementIfFresh()` refuses to hand back a measurement older than
  `VisionConstants.MAX_VISION_AGE_SECONDS` -- a processing delay or a dropped frame
  shouldn't let `DriveTrain` fuse in a pose fix that was accurate a moment ago but
  describes where the robot *was*, not where it *is*. `isAnyVisionAvailable()`
  answers a different, looser question ("has the camera sent anything recently at
  all") used only for dashboard telemetry, not for anything safety- or
  accuracy-critical.

## ApproachTagCommand

`commands/ApproachTagCommand.java` is bound twice in `RobotContainer.java`, once per
requested behavior, both against the same example tag ID
(`VisionConstants.EXAMPLE_TAG_ID = 15`):

```java
driverController.a().onTrue(
    new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, APPROACH_STANDOFF_FEET)
);
driverController.x().onTrue(
    new ApproachTagCommand(
        drivetrain, vision, EXAMPLE_TAG_ID,
        APPROACH_AND_TURN_STANDOFF_FEET, APPROACH_AND_TURN_OFFSET_DEGREES
    )
);
```

Neither binding has a `.withTimeout(...)`, unlike `FireCommand`'s -- see
[Verification status](#verification-status) above for why that's a real gap worth a
rookie noticing.

Both are the same command class, parameterized -- **not** two near-duplicate classes
the way `RaiseElevatorCommand`/`LowerElevatorCommand` are. That's a deliberate
exception to this project's usual "each distinct behavior gets its own named class"
convention, made because these two behaviors share the entire 3-phase state machine
below and differ by exactly one number (`faceOffsetDegrees`, defaulted to `0.0` via a
second, shorter constructor overload -- Java has no default-parameter-value syntax,
so an overload is the idiomatic Java equivalent). Two copies of that state machine would mean any future
bugfix to the turn/drive/turn sequence has to be applied twice and could silently
drift apart; one parameterized class can't drift from itself.

The command runs as three phases, tracked by a private nested `enum Phase` (package-
private in this file specifically for testability -- see
[Running tests](#running-tests) below), each one a fresh PID setpoint on top of
`DriveTrain`'s existing turn/distance PID gains (no separate gains were tuned for
vision navigation -- see
[Assumptions that need bench verification](#assumptions-that-need-bench-verification)):

1. **`TURN_TO_TARGET`** -- `initialize()` looks up the tag's known field pose via
   `vision.getTagPose(tagId)` (a plain field-layout lookup, independent of whether
   the camera can currently see that tag), computes a target point `standoffFeet` in
   front of the tag along the direction the tag itself faces, and turns the robot to
   face that point.
2. **`DRIVE_TO_TARGET`** -- drives straight to that point, using the same
   relative-baseline distance pattern as `DriveDistanceCommand` (see
   [Design decisions](#design-decisions-and-deliberate-simplifications) below for why
   that pattern matters here specifically).
3. **`FACE_TAG`** -- turns to a final heading: facing the tag directly when
   `faceOffsetDegrees` is 0, or offset from that by the given number of degrees
   otherwise. The command finishes once this final turn reaches its setpoint.

If the tag ID doesn't exist in the field layout, `initialize()` sets an internal
`FAILED` phase and the command ends immediately -- it doesn't throw, hang, or drive
toward a nonexistent point.

`ApproachTagCommand`'s constructor calls `addRequirements(drivetrain)` and NOT
`vision` -- it only ever reads from `Vision`, never commands it, and
`addRequirements()` is for preventing two commands from fighting over something they
both *drive*.

## Running tests

```
./gradlew test
```

New in this branch: `VisionTest.java`, `commands/ApproachTagCommandTest.java`, and
one regression test added to `subsystems/DriveTrainTest.java`
(`driveDistanceCommandDoesNotCorruptPoseBetweenLegs`, a regression test for the bug
described in
[Design decisions](#design-decisions-and-deliberate-simplifications) below).

**A second package-private-for-testability exception, and why one exception led to
another.** `DriveTrainTest.java` already needed `DriveTrain.leftEncoder`/
`rightEncoder` to be package-private instead of `private` (see
`teaching-bot-poc-java`'s README for the original reasoning) so a same-package test
could poke simulated encoder position directly. `ApproachTagCommand.java` needed the
same kind of access for its own internal `phase`/`targetPoint`/`turnPid`/`drivePid`
fields, so `ApproachTagCommandTest.java` can check the command's state-machine
progress directly. That test
therefore has to live in `frc.robot.commands` (where `ApproachTagCommand` itself
lives) -- but it *also* needs to poke `DriveTrain`'s encoders to fake a driven
distance mid-test, and package-private access never spans two different packages no
matter how either side is declared: a test can be a member of `frc.robot.commands` or
`frc.robot.subsystems`, never both at once. Rather than force one package-private
trick to do a job it structurally cannot, `DriveTrain` gained one small, honestly
named `public` method, `setEncoderPositionsForTest(double, double)`, whose Javadoc
says exactly what it's for and why it exists. Two real lessons in one small corner
of this codebase: package-private is the standard fix for a same-package test that
needs past Java's `private` enforcement; and package-private has a hard structural
limit -- once a test's need spans two different production packages, a plain
`public` method, clearly labeled, is the more honest tool than trying to bend
visibility rules to fit.

Everything else about running these tests -- `HAL.initialize()`/`DriverStationSim`/
`SimHooks` used explicitly in every file's `@BeforeEach`, no physics simulation
wired into this
project's Gradle build so nothing overwrites a poked sensor value on its own -- is
unchanged from `teaching-bot-poc-java`'s README; see that file for the full
explanation.

## Building and running this project

Identical to `teaching-bot-poc-java` -- see that branch's README for the
`gradle-wrapper.jar` gap and how to fix it in one command.

```
./gradlew build   # compile + run tests
./gradlew simulateJava   # run in WPILib's desktop simulator
```

**The camera is not modeled in simulation.** There's no simulated PhotonVision
camera feed wired into this project (`VisionSystemSim`, a real PhotonLib class for
exactly this, isn't set up here) -- `PhotonCamera.getLatestResult()` returns an
empty, disconnected-camera result under `./gradlew test` or `simulateJava` alike, the
same placeholder behavior `Vision.java`'s own comments describe. `ApproachTagCommand`
will therefore fail immediately (its `FAILED` phase) in simulation unless the tag
pose lookup itself is exercised directly (`vision.getTagPose(...)`, which doesn't
need a camera at all) -- driving the full command end-to-end needs either real
camera hardware or a hand-written test that pokes `Vision`'s state the way
`ApproachTagCommandTest.java` does.

## Design decisions and deliberate simplifications

Identical list to `teaching-bot-poc-java`'s, plus these additions specific to this
branch:

- **Odometry, then vision-fused pose estimation, as two separate branches instead of
  one.** The real competition port's `DriveTrain` fuses encoders, gyro, AND vision
  into a `DifferentialDrivePoseEstimator` from day one. This project deliberately
  split that into `teaching-bot-odometry-java` (encoder/gyro dead reckoning alone)
  and this branch (which upgrades that to a `DifferentialDrivePoseEstimator` fusing
  in AprilTag fixes) -- because dead reckoning and vision correction are two
  separable ideas worth understanding one at a time, and a rookie who can already
  explain *why* dead reckoning drifts gets far more out of learning what vision
  fixes than one meeting both ideas simultaneously.
- **`DriveDistanceCommand` used to reset the encoders every time it ran -- now it
  doesn't, and that fix mattered for a reason that didn't exist before this project
  had odometry.** Resetting the hardware encoders to zero at the start of every
  distance-drive was harmless back when nothing else cared about the encoders'
  absolute reading. Once `DriveTrain` started computing its pose from those same
  encoders every loop, an external reset mid-match silently corrupted the pose
  estimate -- the pose estimator has no way to know a reset happened out from under
  it, so it would compute a wrong, sudden "jump" in position on the very next loop.
  The fix (in both `DriveDistanceCommand` and `ApproachTagCommand`'s drive phase) is
  to capture the *current* distance as a baseline in `initialize()` and measure
  progress relative to that baseline, without ever touching the actual hardware
  encoder. `DriveTrainTest.java`'s `driveDistanceCommandDoesNotCorruptPoseBetweenLegs()`
  is a regression test for exactly this. Worth asking a rookie: what other commands
  in this codebase read a sensor that something else also depends on, and would have
  the same class of bug if they ever called a `reset*()` method on it?
- **`ApproachTagCommand` is one parameterized class, not two, breaking this project's
  usual one-class-per-behavior convention on purpose.** See
  [ApproachTagCommand](#approachtagcommand) above for the full reasoning.
- **A `private enum Phase` became package-private for the same testability reason
  DriveTrain's encoders did, and that need cascaded into DriveTrain gaining a new
  `public` test-only method.** See [Running tests](#running-tests) above -- worth
  reading end to end as one example of how a single design choice (package-private
  test access) can run into a hard structural limit (it can't span two packages) and
  need a second, different kind of fix.

## Assumptions that need bench verification

Identical set to `teaching-bot-poc-java`'s (motor inversions, limit-switch/beam-break
polarity, every PID gain, the shooter's gear ratio), plus these, all marked `TODO` at
their definition in `Constants.VisionConstants`:

- `CAMERA_NAME = "Front_Camera"` -- must match whatever name the camera is actually
  configured with in the PhotonVision coprocessor web UI; a mismatch here means
  `PhotonCamera` silently never finds a live camera, since the constructor doesn't
  fail on an unknown name.
- The camera's mounting measurements themselves (3 inches back from the front, 1
  foot up, 15 degrees up, centered left/right) came from the task description, not a
  tape measure on an actual robot -- `ROBOT_TO_CAMERA` should be re-measured against
  the real mount once one exists, since even a small mounting error compounds into a
  real pose error at range (see [Camera mounting](#camera-mounting) above).
- `ApproachTagCommand` reuses `DriveTrainConstants`' existing turn/distance PID gains
  rather than its own -- those gains were themselves only starting points, and
  driving toward a vision-derived target point may want tighter or looser tolerances
  than driving a human-specified distance/heading in autonomous. Not split out into
  separate constants yet because there's no bench data yet to justify different
  numbers.
- `MAX_TAG_DISTANCE_METERS`, `MAX_AMBIGUITY`, and the three `*_STDDEVS` matrices are
  reasonable-sounding starting points for a PhotonVision AprilTag pipeline, not
  numbers measured against this specific camera/lens/coprocessor combination --
  expect to retune all of them once real vision data exists to compare against a
  known ground-truth position.
- **`ApproachTagCommand` has no `.withTimeout(...)` at either binding** (see
  [Verification status](#verification-status) above) -- if its internal PID chain
  never converges, the driver can't preempt it with the default teleop command.
  It's a real gap worth fixing before this pattern is ever bound on hardware that
  can actually hurt someone. A reasonable fix: wrap both bindings in
  `.withTimeout(...)`, the same decorator `FireCommand`'s binding already uses, with
  a new `VisionConstants` constant for the timeout duration.
- **The `PhotonPoseEstimator` construction pattern itself** (see
  [Verification status](#verification-status) above) -- worth treating as an open
  question to settle, not just a code style preference, once a real build/test
  environment exists: does `estimateCoprocMultiTagPose()`/
  `estimateLowestAmbiguityPose()` actually exist with those exact names in the
  installed `v2026.3.2` PhotonLib Java jar? If not,
  this file needs to move to the competition port's `PoseStrategy`-based pattern
  instead, and this README's framing of that choice was wrong.
- **`DriveTrain.resetPose()` resets the hardware encoders; the real competition
  port's own `resetPose()` doesn't** (see [Verification status](#verification-status)
  above) -- worth deciding deliberately, not by inertia, whether this teaching
  project's behavior or the competition robot's is the one worth adopting if this
  code is ever used as a template for real robot work.

## Blank templates: subsystem and command

Every subsystem in this project follows the same shape (fields for hardware,
a constructor that configures it, optional `periodic()`, plain public methods
for a Command to call); every command follows the same shape too (a
subsystem field, a constructor that calls `addRequirements(...)`, and up to
four lifecycle methods). These two blank templates show that shape on its
own, without any real hardware, annotated section by section -- copy one of
these as a starting point for a new subsystem or command rather than starting
from a blank file.

### BlankSubsystem.java (template)

```java
package frc.robot.subsystems;

// Imports: only import what this subsystem actually uses. At minimum that's
// SubsystemBase; beyond that, typically the vendor/WPILib classes for whatever
// motors/sensors this subsystem owns (see Gripper.java or Elevator.java for real
// examples), plus SmartDashboard if this subsystem reports telemetry.
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: one-sentence description of the real-world mechanism this subsystem
 * controls (what hardware it owns, and in one clause, why it exists).
 *
 * <p>A Subsystem owns exactly one piece of hardware, or one tightly-coupled group
 * of it, and exposes plain methods that Commands call to use it. A Subsystem
 * should never decide WHEN to do something -- that decision belongs to a Command
 * -- only HOW to do it once asked.
 */
public class BlankSubsystem extends SubsystemBase {

    // Fields: one hardware object per field (a motor controller, an encoder, a
    // limit switch, a gyro...), each constructed here at field-declaration time
    // or in the constructor below. Keep them `private` unless a same-package
    // test genuinely needs direct access -- see DriveTrain.java's
    // leftEncoder/rightEncoder fields and their comment for a real example of
    // that exception, and why it's package-private rather than public.

    /**
     * Constructor: construct and configure every hardware object this subsystem
     * owns. Motor inversions, current limits, idle modes, and encoder conversion
     * factors belong here -- configured once, at startup, not repeated every
     * loop in periodic() below.
     */
    public BlankSubsystem() {
        // TODO: construct hardware objects and configure them.
    }

    /**
     * periodic(): called automatically by the CommandScheduler roughly every
     * 20ms, for every subsystem, regardless of whether any command is currently
     * using it. Use it for work that must happen on every loop no matter what --
     * publishing a dashboard value, running a state machine driven by sensor
     * input, or (as in DriveTrain) updating odometry every cycle. Anything that
     * should only happen because a specific command asked for it belongs in a
     * method below, called BY that command -- not here.
     */
    @Override
    public void periodic() {
        // TODO: per-loop bookkeeping, if this subsystem needs any. It's fine to
        // leave this method out entirely (SubsystemBase's default does nothing)
        // if there's no per-loop work to do.
    }

    // Public methods: everything a Command needs in order to actually use this
    // subsystem goes here, as small, mechanical methods -- e.g. setSpeed(double),
    // isAtSetpoint(), a getter for the latest sensor reading. Keep the decision
    // of WHEN to call them out of this class; that belongs in a Command.
}
```

### BlankCommand.java (template)

```java
package frc.robot.commands;

// Imports: WPILib's Command base class, plus whichever subsystem(s) this command
// requires and any math/utility classes its logic needs (a PIDController, for
// example -- see DriveDistanceCommand.java for a real one).
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlankSubsystem;

/**
 * TODO: one-sentence description of the real-world behavior this command
 * produces once it's scheduled.
 *
 * <p>A Command describes WHEN and IN WHAT ORDER to call a Subsystem's methods.
 * The decision-making belongs here; the Subsystem it uses should stay a thin
 * wrapper around hardware.
 */
public class BlankCommand extends Command {

    private final BlankSubsystem subsystem;

    // Additional fields: anything this command needs to remember between loop
    // iterations while it's running -- a target value, a baseline sensor reading
    // captured in initialize() below, a PIDController -- goes here.

    /**
     * Constructor: takes every subsystem and parameter this command needs, saves
     * them, and declares which subsystem(s) it requires with
     * addRequirements(...) -- this is how the CommandScheduler knows to cancel
     * any other command already using the same subsystem before this one starts.
     */
    public BlankCommand(BlankSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    /**
     * initialize(): called exactly once, the instant this command is scheduled.
     * Use it to capture a starting state (a baseline sensor reading, a computed
     * target) or reset anything that needs a clean slate for this particular
     * run. Never do this in the constructor -- a single command instance can be
     * scheduled more than once, and the constructor only runs once, when the
     * command object is created (often long before it's ever scheduled).
     */
    @Override
    public void initialize() {
        // TODO: one-time setup for this run of the command. It's fine to leave
        // this method out entirely if there's genuinely nothing to set up (see
        // EjectCommand.java/IntakeCommand.java for real examples that skip it).
    }

    /**
     * execute(): called every ~20ms while this command is scheduled, after
     * initialize() and before isFinished() is checked each loop. This is where
     * the actual per-loop work happens -- e.g. driving a PID controller toward a
     * setpoint by calling methods on the subsystem.
     */
    @Override
    public void execute() {
        // TODO: per-loop work while this command runs.
    }

    /**
     * isFinished(): checked every loop, right after execute(). Return true the
     * instant this command's job is done; the scheduler then calls end(false)
     * and stops scheduling it. Returning the constant `false` (as written here)
     * makes this a command that never finishes on its own -- correct for
     * something meant to run until interrupted (like TeleopDriveCommand), wrong
     * for anything that should end automatically once a condition is met (like
     * DriveDistanceCommand reaching its target).
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * end(interrupted): called exactly once, either because isFinished()
     * returned true (interrupted == false) or because this command was
     * cancelled or preempted by another command needing the same subsystem
     * (interrupted == true). Use it to leave the subsystem in a safe state --
     * stopping a motor, for example -- regardless of which way the command
     * ended.
     */
    @Override
    public void end(boolean interrupted) {
        // TODO: cleanup that must happen whether this command finished normally
        // or was interrupted.
    }
}
```

## Annotated source code

The code in this repository has had its comments trimmed to near-zero (see each file); this section preserves the original, fully-annotated teaching version of every file for reference.

### src/main/java/frc/robot/Constants.java

```java
package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Robot-wide numerical/boolean constants for the teaching-bot proof of concept.
 *
 * <p>One nested class per subsystem, same convention as the real competition port
 * (2026_competition_code) -- see that repo's Constants.java. Nothing functional lives
 * here, only numbers/IDs.
 *
 * <p>Java has no way to declare a "constant" by naming convention alone: every field
 * needs a declared type ({@code int}, {@code double}, {@code boolean}, ...), and making
 * it an actual, enforced constant needs two keywords together:
 * <ul>
 *   <li>{@code static} -- the field belongs to the CLASS itself, not to any one
 *       instance. Without it, every {@code new DriveTrainConstants()} would get its own
 *       separate copy of {@code TRACK_WIDTH_METERS} -- exactly backwards from what a
 *       shared constant needs. (Nothing in this file is ever actually instantiated with
 *       {@code new} -- these classes exist purely to hold {@code static} fields.)</li>
 *   <li>{@code final} -- once assigned, this field can never be reassigned. This is
 *       what actually makes it a *constant* rather than just a shared variable; leaving
 *       it off would compile fine but silently allow some other file to change
 *       {@code TRACK_WIDTH_METERS} out from under every other file that reads it.</li>
 * </ul>
 * Every field below is {@code public static final}, in that order by convention:
 * access modifier first, then {@code static}, then {@code final}, then the type, then
 * the name.
 *
 * <p>Assumptions worth knowing about (see README for the full list):
 * <ul>
 *   <li>The shooter's Kraken is a Kraken X60. Nothing in this project's simulation
 *       models the shooter at all, so this only matters if someone adds that later.</li>
 *   <li>Every gear ratio, motor inversion, limit-switch polarity, and PID gain below is
 *       a starting guess, marked TODO, meant to be corrected once the real robot exists
 *       to test against.</li>
 * </ul>
 */
public final class Constants {

    // A private constructor with no body is the standard Java idiom for "this class is
    // never meant to be instantiated" -- since every member below is static, a caller
    // never needs a Constants object, only Constants.SomeNestedClass.SOME_FIELD.
    // Marking the constructor private (rather than just leaving the default public one)
    // makes that intent something the compiler enforces, not just something a comment
    // asks nicely for.
    private Constants() {}

    // wpimath (WPILib's own math library) works in meters -- that's not a stylistic
    // choice we get to opt out of, it's baked into DifferentialDriveKinematics,
    // PIDController, etc. But FRC parts, and the humans driving/wrenching on the robot,
    // think in inches/feet. This one constant is the single conversion point between
    // those two worlds: every "feet" value entering this codebase (a command's
    // constructor argument, a dashboard readout) gets multiplied or divided by this,
    // once, right at that boundary -- see commands/DriveDistanceCommand.java and
    // DriveTrain.periodic() for the two places that actually happen.
    public static final double METERS_PER_FOOT = 0.3048;

    public static final class OperatorConstants {
        private OperatorConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveTrainConstants {
        private DriveTrainConstants() {}

        // CAN IDs -- 4x NEO 2.0 via REV SparkMax, 2 per side (lead + follower).
        public static final int LEFT_LEAD_CAN_ID = 20;
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        public static final int RIGHT_LEAD_CAN_ID = 22;
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        public static final int CURRENT_LIMIT = 60; // amps

        // Physical dimensions.
        // 6-wheel "drop center" drivetrain: 3 wheels per side, the center wheel mounted
        // 1/4" LOWER than the front/back wheels. On a rigid frame that means the center
        // wheel touches down first, and only ONE of the front or back wheels shares the
        // ground with it at any moment (whichever end the robot's weight happens to be
        // biased toward) -- never all 3. Effectively, each side only ever has 2 real
        // contact points on the ground, spaced closer together (center-to-front or
        // center-to-back, WHEEL_CENTER_SPACING_METERS) than the full front-to-back
        // wheelbase would be. A shorter ground-contact wheelbase means less wheel scrub
        // (sideways sliding) while turning, which is the entire point of dropping the
        // center wheel: 6-wheel traction/durability for driving straight, without
        // paying a 6-wheel-flat drivetrain's full turning friction penalty. It doesn't
        // change any of the kinematics math below: WPILib's DifferentialDriveKinematics
        // only cares about the distance between the left and right wheels
        // (TRACK_WIDTH_METERS), not how many wheels are on a side or which ones are
        // touching down.
        public static final double DROP_CENTER_WHEEL_DROP_METERS = 0.25 * 0.0254; // 1/4 inch, informational only

        public static final double WHEEL_DIAMETER_METERS = 6.0 * 0.0254; // 6 inches
        public static final double WHEEL_WIDTH_METERS = 1.0 * 0.0254; // 1 inch, informational only
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double GEAR_RATIO = 8.4;

        public static final double TRACK_WIDTH_METERS = 23.0 * 0.0254; // left-to-right wheel center distance
        public static final double WHEEL_CENTER_SPACING_METERS = 13.0 * 0.0254; // front-mid/mid-back spacing, per side

        public static final double ROBOT_LENGTH_METERS = 32.0 * 0.0254; // front-to-back, bumpers included
        public static final double ROBOT_WIDTH_METERS = 28.0 * 0.0254; // side-to-side, bumpers included
        public static final double ROBOT_MASS_KG = 118.0 * 0.45359237; // with bumpers

        public static final double JOYSTICK_DEADBAND = 0.05;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;
        public static final double SPEED_SCALE = 0.7;

        // Gyro-based turning (PID) -- see commands/TurnToAngleCommand.java.
        public static final double TURN_KP = 0.04;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.005;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;

        // Distance-based driving (PID) -- see commands/DriveDistanceCommand.java. KP is
        // deliberately conservative (a large error, e.g. commanding 10 feet from a
        // standstill, would otherwise demand full power) and MAX_OUTPUT clamps the
        // controller's output as a second, independent safety margin on top of that.
        public static final double DRIVE_DISTANCE_KP = 1.5; // TODO: tune on the real robot -- starting point only
        public static final double DRIVE_DISTANCE_KI = 0.0;
        public static final double DRIVE_DISTANCE_KD = 0.1;
        public static final double DRIVE_DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double DRIVE_DISTANCE_MAX_OUTPUT = 0.6; // clamp: never command more than 60% power
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

        // CAN ID: 30s decade, same subsystem-family convention as the competition bot
        // (Shooter/Trigger both feed the same game piece path).
        public static final int FLYWHEEL_MOTOR_ID = 30;

        public static final boolean FLYWHEEL_INVERTED = false; // TODO: verify on bench

        // 5 lb flywheel with 4x 4" compliant wheels as the shooting surface, driven by
        // a Kraken X60.
        public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: confirm -- assumed direct-drive until measured
        public static final double SHOOTER_WHEEL_DIAMETER_METERS = 4.0 * 0.0254;

        public static final int CURRENT_LIMIT = 40; // amps, TalonFX stator limit

        public static final double TARGET_RPM = 3000.0; // TODO: tune once the shooter is built
        public static final double RPM_TOLERANCE = 50.0;

        public static final double SHOOTER_KP = 0.11; // TODO: tune -- starting point only, not measured
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.0;
        public static final double SHOOTER_KV = 0.12;
    }

    public static final class TriggerConstants {
        private TriggerConstants() {}

        // CAN ID: 30s decade, same family as Shooter.
        public static final int CAM_MOTOR_ID = 31;

        public static final boolean CAM_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int CAM_CURRENT_LIMIT = 20; // amps, small NEO
        public static final double CAM_FIRE_SPEED = 0.6; // [-1, 1] duty cycle while firing

        // DIO ports.
        public static final int LIMIT_SWITCH_DIO_PORT = 0;
        public static final int BEAM_BREAK_1_DIO_PORT = 1; // e.g. "ball loaded, waiting to fire"
        public static final int BEAM_BREAK_2_DIO_PORT = 2; // e.g. "ball at the shooter, ready to fire"

        public static final boolean LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench (NC vs NO)
        public static final boolean BEAM_BREAK_1_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BEAM_BREAK_2_INVERTED = false; // TODO: verify polarity on bench

        // Safety timeout in case the limit switch never re-triggers (a jam, a broken
        // wire) -- without this, FireCommand could run the motor forever. Applied as a
        // `.withTimeout()` decorator where FireCommand is bound in RobotContainer.java,
        // not inside the command itself -- see that file for why.
        public static final double FIRE_TIMEOUT_SECONDS = 2.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        // CAN ID: 40s decade -- a new subsystem family, one decade past Shooter/Trigger.
        public static final int LIFT_MOTOR_ID = 40;

        public static final boolean LIFT_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int LIFT_CURRENT_LIMIT = 30; // amps -- Redline motors are small, keep this conservative

        // No encoder on this motor: it's a brushed Redline with no built-in sensor, and
        // no external encoder is installed. This subsystem is entirely open-loop,
        // driven by a limit switch at each end of travel -- see subsystems/Elevator.java.
        public static final double RAISE_SPEED = 0.5; // [-1, 1] duty cycle while raising (spring-assisted)
        public static final double LOWER_SPEED = -0.7; // [-1, 1] duty cycle while lowering (against spring tension)

        public static final int TOP_LIMIT_SWITCH_DIO_PORT = 3;
        public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 4;
        public static final boolean TOP_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BOTTOM_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
    }

    public static final class GripperConstants {
        private GripperConstants() {}

        // CAN ID: 40s decade, same family as Elevator (it rides on the elevator).
        public static final int ROLLER_MOTOR_ID = 41;

        public static final boolean ROLLER_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int ROLLER_CURRENT_LIMIT = 20; // amps, small NEO 550

        public static final double INTAKE_SPEED = 1.0;
        public static final double EJECT_SPEED = -1.0;
    }

    public static final class VisionConstants {
        private VisionConstants() {}

        // Must match the name configured in the PhotonVision UI for this camera.
        public static final String CAMERA_NAME = "Front_Camera";

        // Robot-to-camera mounting transform: new Transform3d(new Translation3d(forward,
        // left, up), new Rotation3d(roll, pitch, yaw)), all relative to the robot's own
        // center at floor level -- the same convention the competition port's
        // ROBOT_TO_FRONT_CAM/ROBOT_TO_REAR_CAM constants use (see that repo's own
        // Constants.java, VisionConstants).
        //
        // Camera is centered left/right (0 lateral offset), mounted 3 inches back from
        // the front of the 32"-long frame -- 13 inches forward of the robot's center,
        // since the center is 16 inches from the front -- 1 foot above the floor,
        // angled 15 degrees upward.
        //
        // Sign convention taken from wpimath's documented Rotation3d pitch behavior, NOT
        // independently verified by running it in this project (see README): rotating
        // Translation3d(1, 0, 0) -- "straight ahead" -- by Rotation3d(0, pitch, 0) is
        // documented to give a NEGATIVE pitch a positive Z component (tilted up) and a
        // POSITIVE pitch a negative Z component (tilted down). "Angled upward" is
        // therefore a negative pitch here -- worth double-checking against whichever
        // WPILib version is installed if this is ever copied elsewhere, since it's easy
        // to get backwards.
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d((16.0 - 3.0) * 0.0254, 0.0, 1.0 * 0.3048),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)
        );

        // Vision measurement quality gating -- same TODO-marked starting points as the
        // competition bot's, not yet tuned against a real camera.
        public static final double MAX_TAG_DISTANCE_METERS = 4.0; // TODO: tune based on camera performance
        public static final double MAX_AMBIGUITY = 0.3; // TODO: tune based on field testing
        public static final int MIN_TAGS_FOR_MULTI_TAG = 2;
        public static final double MAX_VISION_AGE_SECONDS = 0.5;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;

        // Standard deviations for pose estimation, as a 3x1 matrix of (x meters,
        // y meters, heading radians) -- the shape
        // DifferentialDrivePoseEstimator.addVisionMeasurement(...) requires in Java.
        // VecBuilder.fill(...) is the idiomatic way to build one of these from plain
        // numbers without writing out a Matrix constructor by hand -- confirmed against
        // the competition port's own identical use of VecBuilder.fill for this exact
        // field. Multi-tag and close single-tag detections are trusted more (lower std
        // dev) than a single tag far away.
        public static final Matrix<N3, N1> SINGLE_TAG_CLOSE_STDDEVS =
            VecBuilder.fill(0.5, 0.5, Math.toRadians(10)); // TODO: tune based on testing
        public static final Matrix<N3, N1> SINGLE_TAG_FAR_STDDEVS =
            VecBuilder.fill(1.0, 1.0, Math.toRadians(20)); // TODO: tune based on testing
        public static final Matrix<N3, N1> MULTI_TAG_STDDEVS =
            VecBuilder.fill(0.2, 0.2, Math.toRadians(5)); // TODO: tune based on testing

        // The two example ApproachTagCommand bindings in RobotContainer.java -- named
        // here rather than as bare numbers at the binding site, same convention as
        // Auto's routine distances below. Both target the same example tag; nothing
        // about ApproachTagCommand requires that, it's just what "example tag 15" gives
        // us to name concretely.
        public static final int EXAMPLE_TAG_ID = 15;
        public static final double APPROACH_STANDOFF_FEET = 3.0; // stop this far away, facing the tag directly
        public static final double APPROACH_AND_TURN_STANDOFF_FEET = 5.0; // stop this far away, then rotate
        public static final double APPROACH_AND_TURN_OFFSET_DEGREES = 45.0; // positive = right (clockwise) of facing the tag
    }

    public static final class Auto {
        private Auto() {}

        // The two autonomous routines' actual distances/angle, in feet/degrees (the
        // "human" units) -- named here so they're easy to find and change without
        // hunting through autonomous/AutoRoutines.java. Converted to meters only inside
        // DriveDistanceCommand, right at the WPILib-math boundary.
        public static final double DRIVE_FORWARD_ONLY_FEET = 10.0;
        public static final double DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0;
        public static final double DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0; // positive = left (CCW)
        public static final double DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0;
    }
}
```

### src/main/java/frc/robot/Main.java

```java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless
 * you know what you are doing, do not modify this file except to change the parameter
 * class to the startRobot call.
 *
 * <p>{@code public final class Main} with a {@code private Main() {}} constructor is
 * the standard WPILib idiom for "this class is never instantiated, only its
 * {@code main} method is ever called" -- the same {@code private} no-instances pattern
 * used throughout Constants.java, just applied to a class with actual behavior instead
 * of only constants.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>{@code String... args} is Java's "varargs" syntax -- it lets this method be
     * called with any number of String arguments (including zero), collected into a
     * single {@code String[]} inside the method. {@code RobotBase.startRobot(Robot::new)}
     * is what actually builds and runs the robot: {@code Robot::new} is a method
     * reference -- shorthand for "a function that, when called, returns
     * {@code new Robot()}" -- which {@code startRobot} calls internally once it has set
     * up everything a robot program needs (the HAL, the scheduler loop) around it.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
```

### src/main/java/frc/robot/Robot.java

```java
package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.TimedCommandRobot;

/**
 * Entry point for the teaching-bot proof of concept.
 *
 * <p>{@code TimedCommandRobot} does two things a plain {@code TimedRobot} would leave
 * to you: it calls {@code CommandScheduler.getInstance().run()} every loop
 * automatically, and it
 * still gives you the familiar {@code robotInit()}/{@code autonomousInit()}/
 * {@code teleopInit()}/... callback methods to override.
 *
 * <p>No AdvantageKit, no vision-specific logging (unlike the real competition port's
 * {@code Robot.java}, which extends AdvantageKit's {@code LoggedRobot}) -- just
 * {@link DataLogManager} for on-disk + NetworkTables logging, kept deliberately simple
 * for teaching purposes.
 */
public class Robot extends TimedCommandRobot {
    // `Command` (an interface/abstract class) is the TYPE; `m_autonomousCommand` can
    // hold `null` (no autonomous command selected) or any object that implements
    // Command. Java has no separate "nullable" annotation built into the language;
    // ANY non-primitive Java type (anything that isn't
    // `int`/`double`/`boolean`/etc.) can already hold `null`, so `Command` alone is the
    // whole type, and `null` is a value it can take on without any extra syntax.
    private Command m_autonomousCommand;

    // `RobotContainer` is a type WE wrote (see RobotContainer.java) -- Java doesn't
    // distinguish "a class from the standard library" from "a class from this project"
    // in its syntax at all; both are used exactly the same way once imported.
    public RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for
     * any initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        m_robotContainer = new RobotContainer();
    }

    /** This autonomous runs the autonomous command selected by {@link RobotContainer}. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure autonomous stops running when teleop starts. If you want
        // autonomous to continue until interrupted by another command, remove this.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
```

### src/main/java/frc/robot/RobotContainer.java

```java
package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoChooser;
import frc.robot.commands.ApproachTagCommand;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Vision;

/**
 * RobotContainer for the teaching-bot proof of concept.
 *
 * <p>Wires the six subsystems together, sets teleop default commands and button
 * bindings, and builds the autonomous chooser. See README.md for the full
 * controller-binding table and subsystem/command maps -- this file is meant to be read
 * start-to-finish as the map of the whole robot.
 */
public class RobotContainer {

    // `public final` fields (not `private`): unlike every subsystem/command field seen
    // so far, these are deliberately visible outside this class -- Robot.java (and, in
    // a JUnit test, a test class) needs to reach `robotContainer.drivetrain` directly
    // to poke at simulated hardware. `final` still means each is assigned exactly once,
    // right here in the constructor below.
    //
    // `vision` is declared, and constructed, before `drivetrain` -- not just field
    // order for its own sake, but because DriveTrain's constructor needs a real
    // Vision object to hand to `new DriveTrain(vision)` right below it. Java
    // initializes instance fields in the order they're written, top to bottom, so
    // `vision`'s initializer has to come first for `drivetrain`'s initializer to be
    // able to reference it.
    public final Vision vision = new Vision();
    public final DriveTrain drivetrain = new DriveTrain(vision);
    public final Shooter shooter = new Shooter();
    public final Trigger trigger = new Trigger();
    public final Elevator elevator = new Elevator();
    public final Gripper gripper = new Gripper();

    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    /**
     * {@code CommandXboxController} is the current, non-deprecated way to bind
     * buttons to commands for an Xbox-style controller as of WPILib 2026.
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();

        autoChooser = AutoChooser.build(drivetrain);
    }

    private void configureDefaultCommands() {
        // A subsystem's default command runs whenever no other command needs that
        // subsystem -- here, that means "whenever the driver isn't running an
        // autonomous/other DriveTrain command, tank drive from the sticks."
        drivetrain.setDefaultCommand(new TeleopDriveCommand(drivetrain, driverController));
    }

    /**
     * Configure button-to-command bindings.
     *
     * <p>Driver (port 0) -- drive, plus the two example vision commands:
     * <ul>
     *   <li>Left Y / Right Y = tank drive</li>
     *   <li>Back = reset gyro heading to 0 (do this before autonomous!)</li>
     *   <li>A = approach the example tag, stop 3 ft away facing it</li>
     *   <li>X = approach the example tag, stop 5 ft away, then turn 45 degrees
     *       right of facing it</li>
     * </ul>
     *
     * <p>Operator (port 1) -- everything else:
     * <ul>
     *   <li>A = toggle shooter spin-up</li>
     *   <li>B = fire trigger</li>
     *   <li>X = gripper intake while held</li>
     *   <li>Y = gripper eject while held</li>
     *   <li>Right Bumper = raise elevator while held</li>
     *   <li>Left Bumper = lower elevator while held</li>
     * </ul>
     *
     * <p>Every binding below schedules a named Command class -- none of them build a
     * command inline with a lambda, including the one-shot gyro reset (see
     * ResetGyroCommand.java's docstring for why a one-shot action still gets a full
     * class in this project).
     */
    private void configureBindings() {
        driverController.back().onTrue(new ResetGyroCommand(drivetrain));

        driverController.a().onTrue(
            new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, APPROACH_STANDOFF_FEET)
        );
        driverController.x().onTrue(
            new ApproachTagCommand(
                drivetrain,
                vision,
                EXAMPLE_TAG_ID,
                APPROACH_AND_TURN_STANDOFF_FEET,
                APPROACH_AND_TURN_OFFSET_DEGREES
            )
        );

        operatorController.a().toggleOnTrue(new SpinUpShooterCommand(shooter));

        // FireCommand has no timeout of its own -- `.withTimeout()` is a decorator
        // that wraps ANY command (see FireCommand.java's docstring), applied here at
        // the one place this command is actually bound to a button, using the safety
        // timeout defined in TriggerConstants.
        operatorController.b().onTrue(new FireCommand(trigger).withTimeout(FIRE_TIMEOUT_SECONDS));

        operatorController.x().whileTrue(new IntakeCommand(gripper));
        operatorController.y().whileTrue(new EjectCommand(gripper));
        operatorController.rightBumper().whileTrue(new RaiseElevatorCommand(elevator));
        operatorController.leftBumper().whileTrue(new LowerElevatorCommand(elevator));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
```

### src/main/java/frc/robot/VisionMeasurement.java

```java
package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Container for a single vision-based pose measurement from PhotonVision.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's version: one
 * camera means there's no "which of two cameras' measurements do we trust more"
 * arbitration to carry alongside the data, so this only needs what DriveTrain's pose
 * estimator and ApproachTagCommand actually use.
 *
 * <p>{@code record} is a Java feature (since Java 16) purpose-built for exactly this:
 * a small, immutable bundle of named values with no behavior of its own. Writing
 * {@code public record VisionMeasurement(Pose2d estimatedPose, ...)} generates,
 * automatically, everything a hand-written class with the same fields would need --
 * a constructor, a getter for each field (named after the field itself, with no
 * {@code get} prefix -- {@code measurement.estimatedPose()}, not
 * {@code measurement.getEstimatedPose()}), and correct {@code equals()}/
 * {@code hashCode()}/{@code toString()} -- without writing any of that by hand. This
 * project's real competition port ({@code 2026_competition_code}) uses this same
 * {@code record} pattern for its own {@code VisionMeasurement} class, accessed the
 * same parenthesized way ({@code measurement.estimatedPose()},
 * {@code measurement.numTagsUsed()}) -- that usage is where this file's shape was
 * confirmed from, rather than guessed.
 *
 * <p>{@code standardDeviations} is a {@code Matrix<N3, N1>} (a 3-row, 1-column
 * matrix), since Java's
 * {@code DifferentialDrivePoseEstimator.addVisionMeasurement(...)} takes its standard
 * deviations as this typed matrix, built with {@code VecBuilder.fill(x, y, theta)}.
 */
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed
) {}
```

### src/main/java/frc/robot/subsystems/DriveTrain.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.VisionMeasurement;

import java.util.Optional;

/**
 * DriveTrain subsystem -- 6-wheel drop-center differential (tank) drive.
 *
 * <p>Teaching-bot proof of concept.
 *
 * <p>A <b>subsystem</b> in the WPILib command-based framework represents one physical
 * mechanism and owns all the hardware objects for it (motor controllers, sensors). Its
 * job is narrow on purpose: know how to DO things right now (spin the motors at a given
 * power, report what a sensor currently reads) and nothing about WHEN or for HOW LONG
 * to do them. That "when/how long" logic -- including anything with real state, like a
 * PID loop -- lives in commands/, as explicit Command classes that call the plain
 * methods defined below. See the README's "Where do commands live?" section for the
 * full reasoning behind that split, and commands/ for this subsystem's four commands
 * (teleop drive, drive-to-distance, turn-to-angle, reset gyro).
 *
 * <p>{@code SubsystemBase} is the WPILib base
 * class that registers this object with the CommandScheduler and gives it a default,
 * do-nothing {@code periodic()} to override.
 *
 * <p>The {@code teaching-bot-odometry-java} branch this one builds on added
 * {@code DifferentialDriveKinematics} and pure encoder+gyro dead reckoning via
 * {@code DifferentialDriveOdometry} to track the robot's estimated (X, Y, heading)
 * position on the field. This branch replaces that dead reckoning with a
 * {@code DifferentialDrivePoseEstimator}, which does the same encoder+gyro
 * integration AND periodically corrects itself using AprilTag detections from the
 * Vision subsystem ({@code subsystems/Vision.java}) -- exactly the drift-correction
 * the previous branch's README named as the reason to add vision next.
 * {@code DriveTrain} doesn't do any of
 * the actual camera/AprilTag work itself; it just calls
 * {@code vision.getBestVisionMeasurementIfFresh()} every loop and, if there's a
 * fresh one, folds it in via {@code addVisionMeasurement()}. This constructor
 * pattern ({@code DifferentialDrivePoseEstimator(kinematics, gyroAngle,
 * leftDistance, rightDistance, initialPose)} and
 * {@code addVisionMeasurement(pose, timestampSeconds, stdDevs)}) was cross-checked
 * directly against the real competition port's own {@code DriveTrain.java} -- unlike
 * {@code Vision.java}'s {@code PhotonPoseEstimator} construction (see that file's
 * class-level comment), this part of the design was NOT a deliberate divergence.
 */
public class DriveTrain extends SubsystemBase {

    // DriveTrain only ever READS from Vision (asking "got a fresh fix?" every loop)
    // -- it never commands it, so this is a plain reference here, not something
    // DriveTrain calls addRequirements() on. RobotContainer constructs Vision first
    // and passes it in below. `private final`, same as every other field here: once
    // handed a Vision object in the constructor, this field is never reassigned to
    // point at a different one.
    private final Vision vision;

    // ---- Motor groupings: 2 physical motors per side, "lead" + "follower" ----
    //
    // Each side of the drivetrain has 2 NEO 2.0 motors, but we only want to give the
    // software ONE number per side ("drive the left side at 50% power"), not have to
    // command two motors separately and keep them in sync by hand. REV's SparkMax
    // solves this with a lead/follower relationship, configured below in
    // configureMotors(): the "follow" motor is told, once, "always match whatever the
    // lead motor is doing" -- after that, our code only ever talks to the two LEAD
    // motors. The two FOLLOW motors exist as Java objects here only so we can configure
    // them once at startup; nothing in this file calls .set() or reads a sensor from a
    // follower again after the constructor runs.
    //
    // `private final` on every hardware field below: `private` means only this class's
    // own methods can reach it directly (an outside caller has to go through a public
    // method like drive() or getLeftDistanceMeters() instead); `final` means the field
    // is assigned exactly once -- here, right where it's declared -- and can never be
    // reassigned to point at a different SparkMax object afterward. Java's `private`
    // is a real access restriction the compiler checks.
    private final SparkMax leftLead = new SparkMax(LEFT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax leftFollow = new SparkMax(LEFT_FOLLOW_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightLead = new SparkMax(RIGHT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightFollow = new SparkMax(RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);

    // RelativeEncoder objects for the two lead motors. NEOs and NEO 2.0s both have a
    // built-in encoder inside the motor -- no separate sensor to wire up, unlike
    // Elevator's brushed Redline motor (see Elevator.java for that contrast).
    // getEncoder() with no arguments returns the motor's built-in one.
    //
    // PACKAGE-PRIVATE (no access modifier at all), not `private`, and that's a
    // deliberate exception to the `private` rule every other hardware field on this
    // page follows. DriveTrainTest.java (in this same frc.robot.subsystems package)
    // needs to poke these two objects' simulated position directly with
    // `.setPosition(...)` to test odometry and the PID commands without a real robot.
    // Java's `private` is enforced by the compiler with
    // no loophole to reach past, so getting the same test access here needs an actual,
    // coarser access level instead of a bypassable naming hint. Package-private is the
    // narrowest level that still works: any class in frc.robot.subsystems can reach
    // these fields, but nothing outside that package (including RobotContainer.java,
    // in frc.robot) can -- a real, if slightly wider, restriction, not merely a polite
    // request.
    final RelativeEncoder leftEncoder = leftLead.getEncoder();
    final RelativeEncoder rightEncoder = rightLead.getEncoder();

    // The navX2 is a gyroscope (and more -- accelerometer, magnetometer) that plugs
    // into the roboRIO's MXP port and reports over SPI. This is the only sensor on the
    // robot that isn't a motor's built-in encoder or a simple digital switch, which is
    // why it needs its own vendor library (Studica, imported above) instead of coming
    // from `com.revrobotics` or core `edu.wpi.first.wpilibj`.
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // DifferentialDrive is a small WPILib helper that takes "how fast should the
    // left/right side go" and turns that into calls on the two motors it wraps -- it
    // does NOT know about the follower motors at all, because it doesn't need to
    // (that's the whole point of having configured them to follow, below).
    private final DifferentialDrive driver = new DifferentialDrive(leftLead, rightLead);

    // ---- Kinematics and odometry ----
    //
    // DifferentialDriveKinematics only needs one number -- the track width
    // (left-to-right wheel spacing) -- to convert between "each wheel's own speed" and
    // "the whole robot's forward speed and turn rate" (a ChassisSpeeds). It doesn't
    // track anything over time by itself; getChassisSpeeds() below is the only place
    // this project currently uses it.
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // DifferentialDrivePoseEstimator does everything DifferentialDriveOdometry did
    // (integrate gyro heading + encoder distance into a running pose, fed every loop
    // below) PLUS accepts outside corrections via addVisionMeasurement() --
    // internally, it keeps a short history of past poses so a vision fix that
    // arrived slightly late (camera processing takes real time) can be applied at
    // the moment it was actually true, not the moment it was received, then replays
    // odometry forward from there. Constructed here with the encoders already
    // zeroed (see configureMotors() below, called from the constructor before this
    // field is initialized) and the gyro's current heading; starting pose defaulted
    // to Pose2d() (X=0, Y=0, heading=0) -- a stand-in field origin until resetPose()
    // sets a real one, exactly as before.
    private final DifferentialDrivePoseEstimator poseEstimator;

    // Field2d is a Shuffleboard/Glass widget that draws the robot as an icon on a
    // picture of the field, at whatever pose you last gave it -- registered once in
    // the constructor (not every loop) so Shuffleboard doesn't see it
    // appear/disappear/duplicate.
    private final Field2d field = new Field2d();

    // Used by periodic() below to only publish telemetry every Nth loop instead of
    // every ~20ms -- SmartDashboard/NetworkTables traffic adds up, and nothing reads
    // these values fast enough to need them every single loop. Not `final`: this one
    // field IS reassigned, every loop, in periodic() below.
    private int telemetryLoopCounter = 0;

    /**
     * The constructor -- Java calls this automatically for {@code new DriveTrain(vision)}.
     * Unlike the poc-java/odometry-java branches, this constructor now takes one
     * parameter: {@code vision}, the {@code Vision} subsystem instance to read fresh
     * AprilTag fixes from every loop. See commands/TeleopDriveCommand.java for the
     * full explanation of constructor parameter syntax in general (public/type
     * annotations/no return type/implicit super()) -- this comment only covers what's
     * new here.
     *
     * <p>{@code poseEstimator} is assigned here, in the constructor BODY, rather than
     * inline at its field declaration the way {@code kinematics} and {@code field}
     * are above -- it's the one field whose initial value depends on calling
     * {@code getHeadingDegrees()}/{@code getLeftDistanceMeters()}/
     * {@code getRightDistanceMeters()}, which in turn need {@code configureMotors()}
     * to have already zeroed the encoders. Java runs field initializers and the
     * constructor body in the order they're written, top to bottom, so
     * {@code configureMotors()} has to be called first, right here, before
     * {@code poseEstimator} can be built from a known-zero starting state.
     */
    public DriveTrain(Vision vision) {
        this.vision = vision;

        configureMotors();

        poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(getHeadingDegrees()),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            new Pose2d()
        );

        SmartDashboard.putData("Field", field);
    }

    /**
     * One-time SparkMax setup for all 4 drive motors, called once from the
     * constructor. Nothing in here runs again after startup. {@code private}: this is
     * an internal implementation detail, never meant to be called from outside this
     * class -- unlike {@code drive()}/{@code stop()}/the getters below, which are
     * {@code public} because commands need to call them.
     */
    private void configureMotors() {
        // A SparkMaxConfig object describes a full desired configuration; it doesn't
        // take effect until passed to .configure(). This "build a config object, then
        // apply it" two-step (rather than one call per setting) is REVLib's pattern for
        // every SparkMax on this robot, with camelCase method names throughout.
        //
        // positionConversionFactor/velocityConversionFactor rescale the raw "motor
        // shaft rotations" the encoder actually measures into "meters the robot has
        // driven" -- multiplying by wheel circumference accounts for one wheel
        // rotation = one circumference of travel, and dividing by the gear ratio
        // accounts for the motor spinning GEAR_RATIO times for every one wheel
        // rotation. Only the LEAD motors' encoders are configured this way, since those
        // are the only encoders this code ever reads (see the field comments above).
        double conversionFactor = WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO;

        // Right lead. inverted(true) because of how the gearboxes are mounted: a
        // physically-mirrored drivetrain means the left and right gearboxes spin
        // opposite directions for "both sides forward," so exactly one side needs its
        // sign flipped in software.
        SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        rightLeadConfig.inverted(true);
        rightLeadConfig.idleMode(IdleMode.kCoast);
        rightLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        rightLeadConfig.encoder.positionConversionFactor(conversionFactor);
        rightLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        rightLead.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Left lead -- same idea, not inverted.
        SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        leftLeadConfig.inverted(false);
        leftLeadConfig.idleMode(IdleMode.kCoast);
        leftLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        leftLeadConfig.encoder.positionConversionFactor(conversionFactor);
        leftLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        leftLead.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follow motors: `.follow(leadMotor)` is the whole configuration -- this one
        // call is what makes "command the lead, the follower copies it" happen. No
        // encoder conversion factors here, because this code never reads a follower's
        // encoder (see the field comments above).
        SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        leftFollowConfig.follow(leftLead);
        leftFollowConfig.idleMode(IdleMode.kCoast);
        leftFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
        leftFollow.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
        rightFollowConfig.follow(rightLead);
        rightFollowConfig.idleMode(IdleMode.kCoast);
        rightFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
        rightFollow.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Start both encoders at exactly 0 meters traveled. Without this, whatever the
        // encoder happened to read when the robot was last powered off would carry
        // over.
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    // ---- Plain hardware actions (no scheduling, no state machines) ----
    //
    // Everything below is intentionally "dumb": each method does exactly one thing to
    // the hardware, right now, and returns. Commands (in commands/) call these
    // repeatedly, in whatever pattern they need, to build actual robot behavior over
    // time.

    /**
     * Tank-drives at the given left/right duty cycles, each in [-1, 1].
     * {@code MathUtil.applyDeadband} zeroes out small values -- without it, a joystick
     * that doesn't return to <i>exactly</i> 0.0 when released would creep the robot.
     *
     * @param left left-side duty cycle, [-1, 1]
     * @param right right-side duty cycle, [-1, 1]
     */
    public void drive(double left, double right) {
        driver.tankDrive(
            MathUtil.applyDeadband(left, JOYSTICK_DEADBAND),
            MathUtil.applyDeadband(right, JOYSTICK_DEADBAND)
        );
    }

    public void stop() {
        driver.stopMotor();
    }

    // ---- Sensors ----
    //
    // These all return SI units (meters, meters/second, degrees for angle -- there's
    // no "imperial degrees") even though nothing else in FRC is metric. That's
    // deliberate: WPILib's own math expects meters, so keeping this subsystem's
    // internal numbers in meters means it can be handed directly to kinematics/PID
    // code without a conversion at every call site. The conversion to feet (for
    // humans) happens in exactly two places: this file's periodic() telemetry, and
    // commands/DriveDistanceCommand.java's constructor, which is the one place a "how
    // many feet" number enters this subsystem from the outside.

    public double getLeftDistanceMeters() {
        return leftEncoder.getPosition();
    }

    public double getRightDistanceMeters() {
        return rightEncoder.getPosition();
    }

    public double getAverageDistanceMeters() {
        return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
    }

    public double getLeftVelocityMetersPerSecond() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocityMetersPerSecond() {
        return rightEncoder.getVelocity();
    }

    /**
     * Test-only hook: directly sets both drive encoders' simulated position, in
     * meters. {@code leftEncoder}/{@code rightEncoder} above are package-private
     * specifically so a same-package test can poke them directly (see their own
     * comment) -- and that works fine for {@code DriveTrainTest.java}, which lives
     * in this same {@code frc.robot.subsystems} package. But
     * {@code ApproachTagCommandTest.java} needs to fake a driven distance too, and
     * it has to live in {@code frc.robot.commands} instead, to reach
     * {@code ApproachTagCommand}'s own package-private state -- and package-private
     * access never spans two different packages no matter how either side is
     * declared. Rather than pretend that conflict away, this one small `public`
     * method -- its purpose named plainly in its own name -- is the honest fix for
     * a test that genuinely needs to reach across both packages at once.
     */
    public void setEncoderPositionsForTest(double leftMeters, double rightMeters) {
        leftEncoder.setPosition(leftMeters);
        rightEncoder.setPosition(rightMeters);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getHeadingDegrees() {
        // Negated for CCW-positive: the navX reports clockwise-positive by default,
        // but WPILib's convention (and this codebase's) is counterclockwise-positive,
        // so every raw reading gets flipped right here -- the one place that has to
        // know about that mismatch.
        return -gyro.getAngle();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    /**
     * The whole robot's forward speed (m/s) and turn rate (rad/s), computed from the
     * two wheel speeds via {@code DifferentialDriveKinematics}. Nothing in this
     * project currently drives from this -- it's here as the other half of what
     * kinematics is for, alongside odometry.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    // ---- Pose (odometry + vision) ----
    //
    // getPose() is DriveTrain's best current estimate of where the robot is on the
    // field, as a Pose2d (X meters, Y meters, heading). It's still built on the same
    // encoder+gyro dead reckoning as before, but now periodic() also feeds it fresh
    // AprilTag fixes from Vision when they're available, correcting the small errors
    // (wheel scrub, an imperfect track-width measurement) that pure dead reckoning
    // would otherwise accumulate forever.

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Tells the pose estimator "the robot is actually at this pose right now" --
     * used once at the start of autonomous once a starting position is known.
     * Resets the encoders too: distance is measured <i>since the last reset</i>, so
     * an old encoder reading and a freshly reset pose would disagree about where
     * "zero" is.
     */
    public void resetPose(Pose2d pose) {
        resetEncoders();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeadingDegrees()), 0.0, 0.0, pose);
    }

    /**
     * {@code @Override} tells the compiler "this method is meant to replace a method
     * of the same name/signature on the parent class ({@code SubsystemBase})" -- if a
     * typo meant this didn't actually match anything on the parent (e.g.
     * {@code periodc()}), the compiler would flag it as an error instead of silently
     * creating an unrelated new method that never gets called. {@code periodic()} runs
     * every ~20ms for every subsystem, whether or not a command is currently using it
     * -- this is the right place for "always keep this updated" bookkeeping like
     * telemetry, as opposed to logic that should only happen while a specific command
     * is active (that belongs in that command's {@code execute()}).
     */
    @Override
    public void periodic() {
        // Odometry has to be fed every single loop, not just on the slower telemetry
        // schedule below -- skipping updates would mean missing however much the
        // robot moved during the skipped loops, which is exactly the kind of small,
        // silent error that makes dead-reckoned position drift over a match.
        poseEstimator.update(
            Rotation2d.fromDegrees(getHeadingDegrees()), getLeftDistanceMeters(), getRightDistanceMeters()
        );

        // Fold in a fresh AprilTag fix, if Vision has one this loop. This is the
        // actual drift correction: nothing here has to know HOW the measurement was
        // computed, only that it's a (pose, timestamp, confidence) triple the
        // estimator can weigh against its own dead-reckoned belief. `Optional`'s
        // `ifPresent(...)` runs the given block only when a value is actually there.
        Optional<VisionMeasurement> measurement = vision.getBestVisionMeasurementIfFresh();
        measurement.ifPresent(m ->
            poseEstimator.addVisionMeasurement(m.estimatedPose(), m.timestampSeconds(), m.standardDeviations())
        );

        field.setRobotPose(getPose());

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            // Dashboard values are published in feet and feet/sec -- the units a
            // human glancing at Shuffleboard actually thinks in -- even though
            // everything above this point works in meters.
            SmartDashboard.putNumber("DriveTrain/LeftDistFeet", getLeftDistanceMeters() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/RightDistFeet", getRightDistanceMeters() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/LeftVelocityFPS", getLeftVelocityMetersPerSecond() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/RightVelocityFPS", getRightVelocityMetersPerSecond() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/HeadingDeg", getHeadingDegrees());
            Pose2d pose = getPose();
            SmartDashboard.putNumber("DriveTrain/PoseXFeet", pose.getX() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/PoseYFeet", pose.getY() / METERS_PER_FOOT);
        }
    }
}
```

### src/main/java/frc/robot/subsystems/Elevator.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Elevator subsystem -- 2-stage single-mast elevator (AndyMark "Elevator in a Box"
 * style cascade rig), spring-assisted extension, motor+rope retraction.
 *
 * <p>Teaching-bot proof of concept. The Redline motor here is brushed and has no
 * encoder (a real one could add a through-bore/versa encoder later for closed-loop
 * positioning -- see README) -- this subsystem is entirely open-loop, driven only by a
 * limit switch at each end of travel. Raising needs less motor power because the
 * springs are doing most of the work; lowering needs the motor to actively pull the
 * rope in against that same spring tension.
 *
 * <p>This subsystem only exposes plain actions (raise/lower a notch, stop, check the
 * limit switches) -- the "keep raising/lowering while a button is held, but always stop
 * at a limit switch even if the button is still held" behavior lives in
 * commands/RaiseElevatorCommand.java and commands/LowerElevatorCommand.java.
 */
public class Elevator extends SubsystemBase {

    // kBrushed, not kBrushless: a Redline motor has physical brushes (hence the name)
    // and no built-in encoder, unlike every NEO in this project. SparkMax can drive
    // either motor type, but has to be told which one it's talking to, since brushed
    // and brushless motors are commutated (have their windings energized in sequence)
    // completely differently in hardware.
    private final SparkMax liftMotor = new SparkMax(LIFT_MOTOR_ID, MotorType.kBrushed);

    private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_DIO_PORT);

    public Elevator() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.inverted(LIFT_MOTOR_INVERTED);
        liftConfig.idleMode(IdleMode.kBrake);
        liftConfig.smartCurrentLimit(LIFT_CURRENT_LIMIT);
        liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isAtTop() {
        boolean raw = topLimitSwitch.get();
        return TOP_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean isAtBottom() {
        boolean raw = bottomLimitSwitch.get();
        return BOTTOM_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    /** speed is a duty cycle in [-1, 1]: positive raises, negative lowers -- see
     * {@code ElevatorConstants.RAISE_SPEED}/{@code LOWER_SPEED}. */
    public void setSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void stop() {
        liftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
    }
}
```

### src/main/java/frc/robot/subsystems/Gripper.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Gripper subsystem -- spinning roller intake at the end of the elevator.
 *
 * <p>Teaching-bot proof of concept. The simplest subsystem here: one motor, no sensors
 * at all. {@code setSpeed()}/{@code stop()} are the only two things it knows how to do
 * -- commands/IntakeCommand.java and commands/EjectCommand.java just pick which speed
 * to hold while a button is pressed.
 */
public class Gripper extends SubsystemBase {

    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

    /**
     * No parameters besides the implicit {@code this} here -- Gripper doesn't need
     * anything handed to it from the outside to build itself; every value it needs
     * (motor CAN ID, current limit, ...) comes from {@code GripperConstants} instead.
     * Compare this to commands/IntakeCommand.java's constructor, which DOES take a
     * parameter ({@code Gripper gripper}) because a command needs to be told WHICH
     * Gripper object to act on.
     */
    public Gripper() {
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(ROLLER_MOTOR_INVERTED);
        rollerConfig.idleMode(IdleMode.kBrake);
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * {@code speed} is a duty cycle in [-1, 1]: positive intakes, negative ejects -- see
     * {@code GripperConstants.INTAKE_SPEED}/{@code EJECT_SPEED}. The parameter is
     * written {@code double speed} -- Java always puts the type BEFORE the name, with
     * no colon, for every parameter and every field in this project.
     */
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0.0);
    }
}
```

### src/main/java/frc/robot/subsystems/Shooter.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem -- single flywheel (Kraken/TalonFX), fixed target RPM.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's Shooter: no
 * distance-based RPM table (no vision on this robot) -- just a single configurable
 * target speed. Demonstrates Phoenix 6's velocity-control pattern.
 *
 * <p>Worth noting explicitly the first time a rookie hits it: Phoenix 6's Java enum
 * constant names use their own capitalization
 * ({@code InvertedValue.Clockwise_Positive}) rather than the all-caps
 * {@code SNAKE_CASE} Java convention used for enum constants everywhere else in this
 * codebase -- CTRE's own naming choice for this particular library, not a Java
 * language rule.
 *
 * <p>Like the other subsystems, this file only exposes plain hardware actions
 * ({@code setTargetRpm()}, {@code stop()}, the getters) -- the actual Command that uses
 * them lives in commands/SpinUpShooterCommand.java.
 */
public class Shooter extends SubsystemBase {

    // VelocityVoltage and NeutralOut are "control request" objects: instead of calling
    // a method with new arguments every loop (like SparkMax's .set()), Phoenix 6 wants
    // you to build one request object per control mode and re-send it (via
    // setControl(), below) whenever you want to change or refresh what the motor is
    // doing. withSlot(0) picks which of the TalonFX's internal PID gain slots
    // (configured below as slot 0) this velocity request should use.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private double targetRpm = 0.0;

    private final TalonFX flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);

    public Shooter() {
        // Phoenix 6 configuration is one big object built up with chained
        // `.with*()` calls (each one returns the same object back, which is what lets
        // them chain), then applied in one shot via getConfigurator().apply() below --
        // REVLib's SparkMaxConfig from DriveTrain/Trigger/Elevator/Gripper is the same
        // "build a config object, then apply it" idea, just with REV's own
        // method-naming style instead of CTRE's.
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(FLYWHEEL_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
            .withSlot0(
                // Slot0Configs holds the PID(+velocity feedforward) gains the TalonFX
                // itself uses to run its OWN closed velocity loop, in hardware, every
                // control cycle -- much faster than this Java code's ~20ms loop could.
                // This is different from DriveTrain's PID commands, which run the PID
                // math in Java and only send a duty cycle to the motor.
                new Slot0Configs()
                    .withKP(SHOOTER_KP)
                    .withKI(SHOOTER_KI)
                    .withKD(SHOOTER_KD)
                    .withKV(SHOOTER_KV));

        StatusCode configError = flywheelMotor.getConfigurator().apply(flywheelConfig);
        if (!configError.isOK()) {
            DriverStation.reportWarning("Shooter flywheel motor config failed: " + configError, false);
        }
    }

    /**
     * Commands the flywheel to spin at {@code rpm}. Because this is a closed-loop
     * velocity request handled on the TalonFX itself (see the Slot0Configs comment
     * above), this only needs to be called once when the target changes -- not every
     * loop like an open-loop duty cycle motor would need.
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        flywheelMotor.setControl(velocityRequest.withVelocity(rpm / FLYWHEEL_GEAR_RATIO / 60.0));
    }

    public void stop() {
        targetRpm = 0.0;
        flywheelMotor.setControl(neutralRequest);
    }

    public double getCurrentRpm() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0 * FLYWHEEL_GEAR_RATIO;
    }

    public boolean isAtTargetSpeed() {
        return targetRpm > 0 && Math.abs(getCurrentRpm() - targetRpm) <= RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // RPM has no separate "imperial" form the way a distance does, so unlike
        // DriveTrain's telemetry there's no unit conversion to do here.
        SmartDashboard.putNumber("Shooter/CurrentRPM", getCurrentRpm());
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtTargetSpeed());
    }
}
```

### src/main/java/frc/robot/subsystems/Trigger.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.TriggerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Trigger subsystem -- small NEO-driven cam that flicks a game piece into the shooter.
 *
 * <p>Teaching-bot proof of concept. The cam has exactly one sensor: a limit switch that
 * defines its rest ("home") position. This subsystem only exposes plain actions/queries
 * (run the cam motor, read the switch/beam breaks) -- the "run until it's fired one
 * full revolution" logic is real state-machine behavior, so it lives in its own Command
 * class, commands/FireCommand.java, rather than here.
 */
public class Trigger extends SubsystemBase {

    private final SparkMax camMotor = new SparkMax(CAM_MOTOR_ID, MotorType.kBrushless);

    // DigitalInput reads a single digital (on/off) signal from a roboRIO DIO port --
    // the same class WPILib uses for any simple switch or break-beam sensor, since
    // electrically they're the same thing (a circuit that's either open or closed).
    private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput beamBreak1 = new DigitalInput(BEAM_BREAK_1_DIO_PORT);
    private final DigitalInput beamBreak2 = new DigitalInput(BEAM_BREAK_2_DIO_PORT);

    public Trigger() {
        SparkMaxConfig camConfig = new SparkMaxConfig();
        camConfig.inverted(CAM_MOTOR_INVERTED);
        // Brake mode (not DriveTrain's Coast): when the cam motor is commanded to 0, we
        // want it to stop and hold position immediately, not coast -- an idle cam
        // swinging freely could drift off "home" and throw off the next fire cycle's
        // home-switch reading.
        camConfig.idleMode(IdleMode.kBrake);
        camConfig.smartCurrentLimit(CAM_CURRENT_LIMIT);
        camMotor.configure(camConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Every switch/beam-break getter here follows the same shape: read the raw
     * electrical signal, then flip it if that particular sensor's wiring reports
     * {@code true} for the opposite of what we mean (see the {@code *_INVERTED}
     * constants and their TODOs -- this is exactly the kind of thing that must be
     * checked on the real robot, since guessing wrong here silently inverts the
     * sensor's meaning). {@code cond ? a : b} is Java's ternary operator -- "if cond is
     * true, this whole expression's value is a, otherwise it's b".
     */
    public boolean isAtHome() {
        boolean raw = limitSwitch.get();
        return LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage1() {
        boolean raw = beamBreak1.get();
        return BEAM_BREAK_1_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage2() {
        boolean raw = beamBreak2.get();
        return BEAM_BREAK_2_INVERTED ? !raw : raw;
    }

    public void runCam() {
        camMotor.set(CAM_FIRE_SPEED);
    }

    public void stopCam() {
        camMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Trigger/AtHome", isAtHome());
        SmartDashboard.putBoolean("Trigger/BallStage1", hasBallAtStage1());
        SmartDashboard.putBoolean("Trigger/BallStage2", hasBallAtStage2());
    }
}
```

### src/main/java/frc/robot/subsystems/Vision.java

```java
package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionMeasurement;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem -- one PhotonVision camera doing AprilTag pose estimation.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's Vision: one
 * forward-facing camera instead of two (front + rear), so there's no "which camera's
 * measurement do we trust more this loop" arbitration to carry -- just "does the one
 * camera have a usable measurement right now."
 *
 * <p>Like every other subsystem in this project, this one only exposes plain
 * actions/queries ({@code getBestVisionMeasurementIfFresh()}, {@code getTagPose()},
 * {@code isAnyVisionAvailable()}) -- {@code DriveTrain} calls these to fuse a vision
 * fix into its pose estimator (see subsystems/DriveTrain.java), and
 * commands/ApproachTagCommand.java calls {@code getTagPose()} to find out where a
 * specific tag actually is on the field.
 *
 * <p><b>A deliberate difference from the real competition port's own {@code Vision}
 * subsystem, worth flagging explicitly:</b> that file constructs
 * {@code PhotonPoseEstimator} with an explicit
 * {@code PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR} argument and calls a single
 * {@code poseEstimator.update(result)} per camera. This file instead constructs
 * {@code PhotonPoseEstimator} with just the field layout and the camera transform
 * (no strategy argument), and calls the strategy-specific
 * {@code estimateCoprocMultiTagPose(result)} method directly, falling back to
 * {@code estimateLowestAmbiguityPose(result)} when multi-tag PNP data isn't
 * available. Both patterns exist in PhotonLib for the 2026 season, per the vendor
 * docs. <b>This pattern has not been verified against the installed Java
 * {@code v2026.3.2} PhotonLib package</b> -- this file was read and reasoned about,
 * never compiled or run. It names the specific PhotonPoseEstimator methods being
 * exercised, rather than delegating that choice to an opaque {@code PoseStrategy}
 * value, which is a real teaching-clarity reason to prefer it here -- but that is
 * not evidence it compiles or matches the installed jar's actual API. See
 * the README's Verification status section for the full reasoning.
 */
public class Vision extends SubsystemBase {

    // AprilTagFieldLayout is a known, fixed map of tag ID -> field position -- it
    // doesn't need the camera to see anything. Loaded once here so getTagPose() below
    // is a plain lookup, usable even for a tag the camera has never actually detected.
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_CAMERA);

    private double lastResultTimestamp = -1.0;
    private double lastFreshFrameFpgaTimestamp = -1.0;

    // Cached once per loop in periodic(), read by every caller this cycle -- avoids
    // recomputing the same pose estimate multiple times if more than one thing asks
    // for it in the same 20ms. `Optional<VisionMeasurement>` (not a nullable
    // `VisionMeasurement` that might be `null`) is the idiomatic Java way to say "this
    // might not have a value" -- callers are pushed toward handling the empty case
    // explicitly (`.isPresent()`, `.map(...)`, `.orElse(...)`) instead of risking a
    // `NullPointerException` from forgetting to check for `null`.
    private Optional<VisionMeasurement> cachedMeasurement = Optional.empty();

    private int telemetryLoopCounter = 0;

    /**
     * Looks up a tag's known field position, independent of whether the camera
     * currently sees it. {@code Optional<Pose3d>} is empty for an unknown tag ID --
     * {@code AprilTagFieldLayout.getTagPose(int)} itself already returns an
     * {@code Optional}, so this method just passes that straight through.
     */
    public Optional<Pose3d> getTagPose(int tagId) {
        return fieldLayout.getTagPose(tagId);
    }

    /**
     * The best measurement cached this loop, or empty if there wasn't one, or it's
     * older than {@code VisionConstants.MAX_VISION_AGE_SECONDS}.
     */
    public Optional<VisionMeasurement> getBestVisionMeasurementIfFresh() {
        return cachedMeasurement.filter(this::isMeasurementFresh);
    }

    private boolean isMeasurementFresh(VisionMeasurement measurement) {
        return (Timer.getFPGATimestamp() - measurement.timestampSeconds()) <= MAX_VISION_AGE_SECONDS;
    }

    /**
     * True if the camera has produced a fresh frame recently -- not the same as "a
     * tag is currently visible": a connected camera pointed at a blank wall is
     * available but sees nothing.
     */
    public boolean isAnyVisionAvailable() {
        return lastFreshFrameFpgaTimestamp >= 0
            && (Timer.getFPGATimestamp() - lastFreshFrameFpgaTimestamp) <= 0.5;
    }

    private Optional<VisionMeasurement> computeMeasurement(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimatedPose.isEmpty()) {
            estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose pose = estimatedPose.get();
        Pose2d pose2d = pose.estimatedPose.toPose2d();

        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth = fieldLayout.getFieldWidth();
        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth) {
            return Optional.empty(); // a pose off the field is never real
        }

        List<PhotonTrackedTarget> targets = result.getTargets();
        if (targets.size() == 1 && result.getBestTarget().getPoseAmbiguity() > MAX_AMBIGUITY) {
            return Optional.empty();
        }

        int numTags = pose.targetsUsed.size();
        Matrix<N3, N1> stdDevs;
        if (numTags >= MIN_TAGS_FOR_MULTI_TAG) {
            stdDevs = MULTI_TAG_STDDEVS;
        } else {
            double averageDistance = averageTagDistance(targets, pose2d);
            if (averageDistance > MAX_TAG_DISTANCE_METERS) {
                return Optional.empty();
            }
            stdDevs = averageDistance < 2.0 ? SINGLE_TAG_CLOSE_STDDEVS : SINGLE_TAG_FAR_STDDEVS;
        }

        return Optional.of(new VisionMeasurement(pose2d, pose.timestampSeconds, stdDevs, numTags));
    }

    private double averageTagDistance(List<PhotonTrackedTarget> targets, Pose2d robotPose) {
        double totalDistance = 0.0;
        int validTagCount = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                totalDistance += robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
                validTagCount++;
            }
        }
        return validTagCount == 0 ? Double.POSITIVE_INFINITY : totalDistance / validTagCount;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        double timestamp = result.getTimestampSeconds();

        // A camera that has never sent a real result reports a placeholder timestamp
        // near zero (per PhotonLib's documented behavior: -1e-06, not the -1.0 this
        // class starts lastResultTimestamp at -- this exact value has not been
        // confirmed by actually running this Java code, see README) -- the
        // `timestamp > 0` check is what keeps that
        // placeholder from being mistaken for an actual fresh frame the very first
        // time periodic() runs.
        if (timestamp > 0 && timestamp > lastResultTimestamp) {
            lastResultTimestamp = timestamp;
            lastFreshFrameFpgaTimestamp = Timer.getFPGATimestamp();
            cachedMeasurement = computeMeasurement(result);
        }

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            SmartDashboard.putBoolean("Vision/Available", isAnyVisionAvailable());
            SmartDashboard.putBoolean("Vision/HasMeasurement", cachedMeasurement.isPresent());
            cachedMeasurement.ifPresent(
                measurement -> SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.numTagsUsed())
            );
        }
    }
}
```

### src/main/java/frc/robot/commands/ApproachTagCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.Optional;

/**
 * Drives to a point a fixed distance in front of an AprilTag, then turns to face it --
 * optionally rotated some number of degrees off of "directly facing it."
 *
 * <p>This is this project's one genuinely cross-subsystem command: it needs both
 * {@code DriveTrain} (to actually move) and {@code Vision} (to know where a tag is).
 * Earlier branches of this project explicitly said no command needed more than one
 * subsystem -- that was true until this one. Decisions like that are provisional, not
 * permanent: once the situation changes (adding vision), the right response is to
 * update the decision, not defend it. See the README's "Where do commands live?"
 * section for the fuller version of this note.
 *
 * <p>This constructor only calls {@code addRequirements(drivetrain)}, not {@code vision}
 * -- it only ever READS from Vision (asking "where is this tag?"), never commands it,
 * and {@code addRequirements()} exists to prevent two commands from fighting over
 * something they both DRIVE, not something they both read.
 *
 * <p>Three phases, run one after another inside this single command (the same kind of
 * internal state machine as {@code FireCommand}, just with three states instead of
 * two -- a plain {@code enum} here rather than {@code FireCommand}'s boolean flag,
 * since there are more than two states to name):
 *
 * <ol>
 *   <li>{@code TURN_TO_TARGET} -- turn in place to face the point we're driving to.
 *   <li>{@code DRIVE_TO_TARGET} -- drive straight to that point.
 *   <li>{@code FACE_TAG} -- turn in place to the final heading.
 * </ol>
 *
 * <p>Nothing about <i>where</i> to go is known until this command actually starts:
 * {@code initialize()} reads the robot's current estimated pose (from
 * {@code DriveTrain.getPose()}) and the tag's known field position (from
 * {@code Vision.getTagPose()}) to compute the target point and final heading, the
 * same loop this command is scheduled -- unlike {@code DriveDistanceCommand}/
 * {@code TurnToAngleCommand}, whose targets are fixed numbers known when they're
 * constructed.
 *
 * <p>Why one parameterized class instead of two nearly-identical ones (the way
 * {@code RaiseElevatorCommand}/{@code LowerElevatorCommand} are two separate classes
 * for what's conceptually "the same page, in the other direction"): those two
 * commands' bodies are almost entirely different numbers, three lines each. This
 * command's three phases -- computing a target point, turning to a bearing, driving a
 * distance, turning to a final heading -- are the same ~90 lines of state-machine
 * logic for both of this project's example uses; the only thing that differs between
 * "stop 3 feet away, facing the tag" and "stop 5 feet away, then turn 45 degrees off
 * of facing it" is one number, {@code faceOffsetDegrees}. Duplicating that logic to
 * keep two separate classes would risk the two copies drifting out of sync the next
 * time one gets a bugfix -- see RobotContainer.java for the two named instances this
 * project actually binds.
 */
public class ApproachTagCommand extends Command {

    // PACKAGE-PRIVATE (no access modifier), not `private` -- the same deliberate
    // exception documented on DriveTrain's leftEncoder/rightEncoder fields
    // (subsystems/DriveTrain.java). ApproachTagCommandTest.java (in this same
    // frc.robot.commands package) needs to read `phase`/`targetPoint`/`turnPid`/
    // `drivePid` directly to check this command's state-machine progress. Java's
    // `private` has no loophole to reach past, so getting the same test access here
    // needs an actual, coarser access level -- package-private is the narrowest one
    // that still works.
    enum Phase {
        TURN_TO_TARGET,
        DRIVE_TO_TARGET,
        FACE_TAG,
        FAILED,
        DONE
    }

    private final DriveTrain drivetrain;
    private final Vision vision;
    private final int tagId;
    private final double standoffMeters;

    // Positive faceOffsetDegrees means "turn right (clockwise) from facing the tag
    // directly" -- the opposite sign from this codebase's CCW-positive convention, so
    // it's negated once, right here, at the boundary where a human-facing "turn
    // right" number enters this command.
    private final double faceOffsetDegrees;

    // Reuses DriveTrain's own turn/drive PID gains rather than introducing a second,
    // separately-tuned set -- this command does the exact same two kinds of motion
    // (turn in place, drive straight) that TurnToAngleCommand/DriveDistanceCommand
    // already tune gains for. Package-private for the same test-access reason as
    // `Phase` above.
    final PIDController turnPid;
    final PIDController drivePid;

    Phase phase = Phase.DONE;
    Translation2d targetPoint = new Translation2d();
    private double finalHeadingDegrees = 0.0;
    private double driveStartDistanceMeters = 0.0;

    /**
     * @param drivetrain the subsystem this command drives -- passed to
     *     {@code addRequirements()} so the scheduler knows this command owns it
     * @param vision read-only: used to look up {@code tagId}'s known field position
     * @param tagId which AprilTag to approach
     * @param standoffFeet how far from the tag to stop, in feet, measured along the
     *     tag's own facing direction
     * @param faceOffsetDegrees how many degrees off of "directly facing the tag" the
     *     final heading should be, positive = clockwise (right)
     */
    public ApproachTagCommand(
        DriveTrain drivetrain, Vision vision, int tagId, double standoffFeet, double faceOffsetDegrees
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagId = tagId;
        this.standoffMeters = standoffFeet * METERS_PER_FOOT;
        this.faceOffsetDegrees = -faceOffsetDegrees;
        addRequirements(drivetrain);

        turnPid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        turnPid.enableContinuousInput(-180, 180);
        turnPid.setTolerance(TURN_TOLERANCE_DEGREES);

        drivePid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        drivePid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    /**
     * Overload matching this project's two actual bindings (both use
     * {@code faceOffsetDegrees = 0.0}, "stop facing the tag directly") -- Java has no
     * default-parameter-value syntax, so an overload is the idiomatic Java equivalent: a
     * second, shorter constructor that just calls the full one with a fixed value for
     * the argument callers usually don't need to give.
     */
    public ApproachTagCommand(DriveTrain drivetrain, Vision vision, int tagId, double standoffFeet) {
        this(drivetrain, vision, tagId, standoffFeet, 0.0);
    }

    @Override
    public void initialize() {
        Optional<Pose3d> tagPose = vision.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            // An unknown tag ID -- nothing to approach. Ending immediately (isFinished()
            // checks for this phase) is safer than guessing; see the README's
            // "Assumptions that need bench verification" for why this is worth a
            // dashboard warning in a real robot, not just a silently-do-nothing command.
            phase = Phase.FAILED;
            return;
        }

        Pose2d tagPose2d = tagPose.get().toPose2d();
        // A tag's pose "faces" outward, away from the tag surface -- the standoff
        // point is that far along the tag's own facing direction, and facing the tag
        // from there means pointing the opposite way (180 degrees from how the tag
        // itself faces).
        Rotation2d tagFacing = tagPose2d.getRotation();
        targetPoint = tagPose2d.getTranslation()
            .plus(new Translation2d(standoffMeters, 0.0).rotateBy(tagFacing));
        finalHeadingDegrees = tagFacing.plus(Rotation2d.fromDegrees(180.0 + faceOffsetDegrees)).getDegrees();

        beginTurnToTarget();
    }

    private void beginTurnToTarget() {
        Translation2d currentTranslation = drivetrain.getPose().getTranslation();
        Translation2d delta = targetPoint.minus(currentTranslation);
        double bearingDegrees = Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));

        turnPid.reset();
        turnPid.setSetpoint(bearingDegrees);
        phase = Phase.TURN_TO_TARGET;
    }

    private void beginDriveToTarget() {
        // Distance-to-go is recomputed here, once, from the pose the robot actually
        // ended the turn at -- not assumed from the turn's own target, since a real
        // turn won't land exactly on its setpoint.
        double distanceMeters = drivetrain.getPose().getTranslation().getDistance(targetPoint);
        driveStartDistanceMeters = drivetrain.getAverageDistanceMeters();

        drivePid.reset();
        drivePid.setSetpoint(distanceMeters);
        phase = Phase.DRIVE_TO_TARGET;
    }

    private void beginFaceTag() {
        turnPid.reset();
        turnPid.setSetpoint(finalHeadingDegrees);
        phase = Phase.FACE_TAG;
    }

    @Override
    public void execute() {
        switch (phase) {
            case TURN_TO_TARGET -> {
                double output = turnPid.calculate(drivetrain.getHeadingDegrees());
                drivetrain.drive(-output, output);
                if (turnPid.atSetpoint()) {
                    beginDriveToTarget();
                }
            }
            case DRIVE_TO_TARGET -> {
                // Same relative-distance measurement DriveDistanceCommand uses -- see
                // that command's initialize() doc comment for why this can't just
                // reset the encoders to zero instead.
                double distanceThisPhase = drivetrain.getAverageDistanceMeters() - driveStartDistanceMeters;
                double output = drivePid.calculate(distanceThisPhase);
                output = Math.max(-DRIVE_DISTANCE_MAX_OUTPUT, Math.min(DRIVE_DISTANCE_MAX_OUTPUT, output));
                drivetrain.drive(output, output);
                if (drivePid.atSetpoint()) {
                    beginFaceTag();
                }
            }
            case FACE_TAG -> {
                double output = turnPid.calculate(drivetrain.getHeadingDegrees());
                drivetrain.drive(-output, output);
            }
            case FAILED, DONE -> {
                // Nothing to do -- isFinished() ends the command.
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (phase == Phase.FAILED || phase == Phase.DONE) {
            return true;
        }
        return phase == Phase.FACE_TAG && turnPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
```

### src/main/java/frc/robot/commands/DriveDistanceCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Drives straight to a target distance, given in FEET, using a PID loop on the average
 * of the two drive encoders.
 *
 * <p>Why PID instead of "drive at a fixed speed until the encoder says you're far
 * enough" (this command's first version): a fixed speed either overshoots -- the
 * motors are still at full speed right up to the exact instant the target is crossed,
 * so the robot coasts/slams past it -- or forces someone to guess a "stop early to
 * leave room for coasting" fudge factor. A PID controller instead recalculates "how
 * hard should I push" every loop from how much distance is left, so the commanded
 * speed naturally tapers off as the target gets close instead of being all full power
 * then all stop.
 */
public class DriveDistanceCommand extends Command {

    private final DriveTrain drivetrain;
    // `double targetDistanceMeters` and `PIDController pid` are both computed/built in
    // the constructor and then never reassigned, so both could be declared `final`
    // exactly like `drivetrain` above -- they're written without `final` here only
    // because the constructor computes/builds them from a parameter rather than
    // receiving them directly, which is a distinction of readability preference, not
    // one Java's compiler cares about.
    private final double targetDistanceMeters;
    private final PIDController pid;

    // Not `final`: set fresh every time this command starts, in initialize() below --
    // see that method's doc comment for why this field exists at all.
    private double startDistanceMeters;

    /**
     * {@code double distanceFeet} -- the same {@code name: type} idea as every other
     * parameter in this project, just using one of Java's built-in primitive types
     * ({@code double}, a 64-bit decimal number) instead of a class like
     * {@code DriveTrain}. Feet-to-meters conversion happens exactly once, right here,
     * at the boundary where a "human" feet value enters this command -- see
     * {@code Constants.METERS_PER_FOOT}.
     */
    public DriveDistanceCommand(DriveTrain drivetrain, double distanceFeet) {
        this.drivetrain = drivetrain;
        this.targetDistanceMeters = distanceFeet * METERS_PER_FOOT;
        addRequirements(drivetrain);

        pid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        pid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    @Override
    public void initialize() {
        // initialize() runs exactly once, the instant this command is scheduled (not
        // when it's constructed in the constructor, which for autonomous commands
        // happens once at RobotContainer startup, possibly minutes before the command
        // actually runs).
        //
        // This records the CURRENT encoder reading as a baseline rather than calling
        // drivetrain.resetEncoders() to zero it -- a tempting shortcut this command
        // used to take, until DriveTrain grew a pose estimator (see
        // subsystems/DriveTrain.java) that reads these same encoders every loop.
        // Odometry measures distance *since its last reset*, so zeroing the encoders
        // out from under it looks exactly like the robot teleporting back near the
        // origin -- a real bug this command caused for every leg after the first in
        // the drive-turn-drive autonomous routine, since fixed by measuring a
        // relative distance instead of an absolute one.
        startDistanceMeters = drivetrain.getAverageDistanceMeters();
        pid.reset();
        pid.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void execute() {
        // PIDController.calculate(measurement) returns "how hard to push" based on the
        // error between `measurement` and the setpoint given in initialize(). It's
        // clamped to +/-DRIVE_DISTANCE_MAX_OUTPUT as a second, independent safety
        // margin on top of tuning KP conservatively -- a large distance error
        // (commanding 10 feet from a dead stop) should never be able to demand more
        // than that fraction of full power.
        double distanceThisLeg = drivetrain.getAverageDistanceMeters() - startDistanceMeters;
        double output = pid.calculate(distanceThisLeg);
        output = Math.max(-DRIVE_DISTANCE_MAX_OUTPUT, Math.min(DRIVE_DISTANCE_MAX_OUTPUT, output));
        drivetrain.drive(output, output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    /**
     * {@code boolean interrupted} -- the CommandScheduler fills this parameter in for
     * you: {@code true} if this command got cut off early (the driver grabbed the
     * joystick mid-autonomous, the match ended, the robot got disabled), {@code false}
     * if {@code isFinished()} returned {@code true} on its own. {@code end()} runs
     * exactly once either way, which is exactly why stopping the motors belongs here
     * rather than only handling the "finished normally" path -- this command doesn't
     * need to tell the two cases apart, but {@code end()} always receives this
     * parameter regardless of whether a command reads it.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
```

### src/main/java/frc/robot/commands/EjectCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/** Same shape as IntakeCommand.java, opposite direction -- see that file for the full
 * explanation of this class's constructor and lifecycle methods. */
public class EjectCommand extends Command {

    private final Gripper gripper;

    public EjectCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpeed(EJECT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
```

### src/main/java/frc/robot/commands/FireCommand.java

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

/**
 * FireCommand is the one genuinely tricky piece of logic in this whole codebase, and
 * it's a good example of why "just read the one sensor" isn't always enough: the cam
 * has exactly one sensor, a limit switch at "home," and reading it once at the start
 * would immediately (and wrongly) report "done," since the cam starts each fire cycle
 * already at home. A full fire cycle actually means: leave home, THEN come back to
 * home. {@code isFinished()} below tracks that as one bit of state,
 * {@code hasLeftHome}, which is reset every time this command starts over in
 * {@code initialize()}.
 *
 * <p>This class does NOT set its own timeout. A jammed cam or a broken switch wire
 * means it would never see "returned home" and would run forever on its own; the
 * safety timeout is applied as a {@code .withTimeout()} decorator at the one place
 * this command is actually bound to a button, in RobotContainer.java. Decorators like
 * {@code .withTimeout()}/{@code .andThen()}/{@code .until()} work on ANY Command -- an
 * explicit class like this one just as well as a {@code Commands.run(...)} one-liner --
 * which is why it doesn't matter that FireCommand and, say, DriveDistanceCommand build
 * their behavior in very different ways internally.
 */
public class FireCommand extends Command {

    private final Trigger trigger;
    // Not `final`: unlike every field seen so far, this one IS reassigned after
    // construction -- once in initialize() every time the command restarts, and again
    // inside isFinished() as the cam leaves home. It's a plain boolean instance field,
    // not a parameter -- nothing external ever passes this in; it's a value this
    // object tracks purely for itself.
    private boolean hasLeftHome = false;

    public FireCommand(Trigger trigger) {
        this.trigger = trigger;
        addRequirements(trigger);
    }

    @Override
    public void initialize() {
        hasLeftHome = false;
    }

    @Override
    public void execute() {
        trigger.runCam();
    }

    @Override
    public boolean isFinished() {
        if (!hasLeftHome) {
            // Still waiting for the cam to leave home for the first time -- once it
            // does, remember that and start watching for it to come back.
            if (!trigger.isAtHome()) {
                hasLeftHome = true;
            }
            return false;
        }
        return trigger.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        trigger.stopCam();
    }
}
```

### src/main/java/frc/robot/commands/IntakeCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/**
 * The simplest command in this project: hold a fixed speed while bound (whileTrue in
 * RobotContainer.java), stop when released. No sensors, no looping math -- a good
 * first command file to read.
 */
public class IntakeCommand extends Command {

    private final Gripper gripper;

    /**
     * See TeleopDriveCommand.java's constructor for the full explanation of every
     * piece of syntax here: {@code public}, the {@code Gripper gripper} parameter's
     * mandatory type, the implicit no-arg {@code super()} call Java inserts since none
     * is written, and why a constructor is written with no return type at all (not
     * even {@code void}).
     */
    public IntakeCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpeed(INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        // Always false: this command is meant to be bound with whileTrue, so it only
        // stops when the button is released, which the scheduler handles by calling
        // end() below instead.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // See DriveDistanceCommand.java's end() for what the `interrupted` parameter
        // means -- this command doesn't need to look at its value, but every command's
        // end() method receives it regardless.
        gripper.stop();
    }
}
```

### src/main/java/frc/robot/commands/LowerElevatorCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.LOWER_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/** See RaiseElevatorCommand.java for the shared reasoning behind this pair. */
public class LowerElevatorCommand extends Command {

    private final Elevator elevator;

    public LowerElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtBottom()) {
            elevator.stop();
        } else {
            elevator.setSpeed(LOWER_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
```

### src/main/java/frc/robot/commands/RaiseElevatorCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.RAISE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * RaiseElevatorCommand and LowerElevatorCommand (in LowerElevatorCommand.java) are
 * near-identical on purpose: both are meant to be bound with whileTrue (see
 * RobotContainer.java), so {@code isFinished()} always returns {@code false} and the
 * command only stops when the button is released (which interrupts it, calling
 * {@code end()}) or when {@code execute()} itself detects a limit switch and calls
 * {@code stop()}. That second check matters even though the command is also
 * "supposed" to stop when the button is released: it protects the mechanism the
 * instant it reaches a limit, without waiting on the operator to notice and let go.
 */
public class RaiseElevatorCommand extends Command {

    private final Elevator elevator;

    public RaiseElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtTop()) {
            elevator.stop();
        } else {
            elevator.setSpeed(RAISE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
```

### src/main/java/frc/robot/commands/ResetGyroCommand.java

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Resets the navX gyro's heading to 0 -- bound to the driver's Back button. Do this
 * before every autonomous run, with the robot pointed the way it should be for that
 * run's "0 degrees."
 *
 * <p>A one-shot action still gets a full class here, on purpose (see the README's
 * "Where do commands live?" section): {@code initialize()} does the actual work, and
 * {@code isFinished()} returns {@code true} immediately so the command scheduler ends
 * it the very next loop after that -- there's no {@code execute()} at all, since
 * there's nothing to repeat.
 */
public class ResetGyroCommand extends Command {

    private final DriveTrain drivetrain;

    /**
     * Same constructor pattern as TeleopDriveCommand.java, just with one parameter
     * instead of two, and no explicit {@code super(...)} call needed (see that file's
     * constructor for the full explanation of both).
     */
    public ResetGyroCommand(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
```

### src/main/java/frc/robot/commands/SpinUpShooterCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.TARGET_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Only one command for Shooter: spin the flywheel up to a fixed target speed, and hold
 * it there until interrupted. Notice this class has no {@code execute()} at all --
 * that's not an omission. Phoenix 6's velocity control is closed-loop on the TalonFX
 * itself (see subsystems/Shooter.java), so this command only has to say "go to this
 * speed" once ({@code initialize()}) and "stop" once ({@code end()}) -- there's nothing
 * to redo every 20ms loop the way DriveTrain's open-loop, duty-cycle commands need.
 */
public class SpinUpShooterCommand extends Command {

    private final Shooter shooter;

    public SpinUpShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(TARGET_RPM);
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (the operator presses the toggle button again -- see
        // RobotContainer.java's toggleOnTrue binding).
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
```

### src/main/java/frc/robot/commands/TeleopDriveCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.SPEED_SCALE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

/**
 * The default command: tank drive read straight from the driver controller's two
 * joystick Y-axes.
 *
 * <p>Takes the controller object itself, rather than two "give me the current
 * left/right stick value" suppliers -- a common alternative
 * ({@code DoubleSupplier} arguments filled in with a lambda like
 * {@code () -> -controller.getLeftY()} at the call site) that avoids naming this class
 * but requires understanding lambdas/method references to read. Calling
 * {@code driverController.getLeftY()} directly, right here, is one idea instead of two.
 *
 * <p>This is the simplest possible Command with real behavior. It has no state and
 * nothing to set up or clean up, so it only overrides {@code execute()} and
 * {@code isFinished()} -- there's no need to write empty {@code initialize()}/
 * {@code end()} methods just to have them; {@code Command}'s base class already
 * provides do-nothing versions. {@code isFinished()} always returns {@code false}
 * because a default command is meant to run forever, until some other command needs
 * DriveTrain and interrupts it.
 */
public class TeleopDriveCommand extends Command {

    // `private final` fields, one per constructor parameter, assigned once in the
    // constructor and never reassigned afterward -- see the constructor below for the
    // full explanation of why these exist and what each piece of the constructor's
    // signature means.
    private final DriveTrain drivetrain;
    private final CommandXboxController driverController;

    /**
     * The constructor. Java calls this automatically whenever something writes
     * {@code new TeleopDriveCommand(...)} -- in this project, that happens exactly
     * once, in RobotContainer.java's {@code configureDefaultCommands()}. Several
     * pieces of syntax on this line are worth calling out individually, since they
     * repeat, in different combinations, in every command class in this project:
     *
     * <ul>
     *   <li><b>{@code public}</b> -- an access modifier. It means any other class,
     *       anywhere in this project (or beyond), can call {@code new
     *       TeleopDriveCommand(...)}. Compare this to {@code private} fields like
     *       {@code drivetrain} above: those can only be read by code written inside
     *       this very class. Java requires an explicit access modifier decision like
     *       this on every field and method, enforced by the compiler.</li>
     *   <li><b>{@code DriveTrain drivetrain}</b> -- a parameter. Java requires every
     *       parameter to have a declared type, written
     *       BEFORE the name with no colon, and the compiler itself refuses to compile
     *       code that passes the wrong type here.</li>
     *   <li><b>{@code CommandXboxController driverController}</b> -- the physical Xbox
     *       controller plugged into port 0 (see {@code Constants.OperatorConstants
     *       .DRIVER_CONTROLLER_PORT}, and RobotContainer.java, where the real
     *       controller object is actually constructed and passed in here). Storing the
     *       whole controller object -- instead of, say, two numbers read from it once
     *       -- is what lets {@code execute()} below call {@code .getLeftY()}/
     *       {@code .getRightY()} on it fresh every single loop.</li>
     *   <li><b>No return type written before the constructor's name at all</b> -- not
     *       even {@code void}. Every normal Java method needs a return type
     *       ({@code void} for "returns nothing," or a real type for "returns this").
     *       A constructor is the one exception: it implicitly builds and returns the
     *       new object, and Java's grammar does not allow ANY return-type keyword to
     *       be written on this line, {@code void} included -- writing one is a syntax
     *       error, not just bad style. A constructor
     *       is a distinct kind of member with its own grammar rule forbidding a return
     *       type outright.</li>
     *   <li><b>{@code super(); }-- wait, there is no {@code super()} call written
     *       here.</b> In Java, if a
     *       constructor's first line does NOT explicitly call {@code super(...)}, the
     *       compiler automatically inserts a call to the parent class's no-argument
     *       constructor for you, as if it were the first line. {@code Command}'s own
     *       no-argument constructor does the setup this class needs, so nothing
     *       explicit is required here.</li>
     * </ul>
     */
    public TeleopDriveCommand(DriveTrain drivetrain, CommandXboxController driverController) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        // `this.drivetrain = drivetrain;` -- the field and the parameter share the
        // same name on purpose (this project's Java convention), which means `this.` in front of
        // the left-hand side is not optional decoration here: without it, `drivetrain
        // = drivetrain;` would just assign the parameter to itself and leave the
        // field permanently unset. `this.` explicitly means "the field belonging to
        // the object being constructed," disambiguating it from the same-named
        // parameter.
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // execute() runs every ~20ms while this command is scheduled -- exactly often
        // enough to keep reading fresh joystick values and keep driving. Xbox
        // joysticks report "pushed forward" as a NEGATIVE Y value, which is backwards
        // from how a driver thinks about "forward" -- the leading minus signs below
        // flip that back.
        double leftY = -driverController.getLeftY();
        double rightY = -driverController.getRightY();
        drivetrain.drive(SPEED_SCALE * leftY, SPEED_SCALE * rightY);
    }

    @Override
    public boolean isFinished() {
        // `boolean` (lowercase) is one of Java's eight built-in "primitive" types --
        // unlike `DriveTrain` or `CommandXboxController` above, it is not a class, has
        // no methods of its own, and can only ever hold `true` or `false`, never
        // `null`. Returning `false` here means "never finish on your own" -- exactly
        // what a default command needs, since it's meant to keep running until some
        // OTHER command that also needs DriveTrain gets scheduled and interrupts this
        // one instead.
        return false;
    }
}
```

### src/main/java/frc/robot/commands/TurnToAngleCommand.java

```java
package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * PID-turns to an absolute heading, given in degrees (CCW-positive, 0 = whatever
 * heading the navX gyro was last reset to). Positive = turn left.
 */
public class TurnToAngleCommand extends Command {

    private final DriveTrain drivetrain;
    private final double targetDegrees;
    private final PIDController pid;

    public TurnToAngleCommand(DriveTrain drivetrain, double targetDegrees) {
        this.drivetrain = drivetrain;
        this.targetDegrees = targetDegrees;
        addRequirements(drivetrain);

        pid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        // A heading wraps around at +/-180 degrees. Without this next line, turning
        // from 179 degrees to -179 degrees (really just a 2-degree turn) would look to
        // a naive PID controller like a 358-degree turn the "long way around."
        // enableContinuousInput tells it to treat the range as a circle instead of a
        // straight line.
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(TURN_TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(targetDegrees);
    }

    @Override
    public void execute() {
        double output = pid.calculate(drivetrain.getHeadingDegrees());
        // Turning in place: equal and opposite power to each side. Positive output
        // steers left, so the left side goes backward while the right side goes
        // forward.
        drivetrain.drive(-output, output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
```

### src/main/java/frc/robot/autonomous/AutoChooser.java

```java
package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

/**
 * Builds the SmartDashboard autonomous-routine chooser for the teaching-bot proof of
 * concept: two real routines plus a "Do Nothing" default.
 */
public final class AutoChooser {
    private AutoChooser() {}

    private static final String DO_NOTHING_NAME = "Do Nothing";
    private static final String DRIVE_FORWARD_NAME = "Drive Forward 10 ft";
    private static final String DRIVE_TURN_DRIVE_NAME = "Drive 5ft, Turn Left 90, Drive 3ft";

    /**
     * {@code SendableChooser<Command>} -- the angle brackets are a Java GENERIC type
     * parameter: this says "a SendableChooser whose options are all Command objects."
     * Java's compiler uses the {@code <Command>}
     * to guarantee every option ever added to (or read from) this specific chooser
     * really is a Command, catching a wrong-type mistake at compile time instead of
     * only when the mistaken value is actually used.
     */
    public static SendableChooser<Command> build(DriveTrain drivetrain) {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption(DO_NOTHING_NAME, Commands.none());
        chooser.addOption(DRIVE_FORWARD_NAME, AutoRoutines.driveForwardOnly(drivetrain));
        chooser.addOption(DRIVE_TURN_DRIVE_NAME, AutoRoutines.driveTurnDrive(drivetrain));
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }
}
```

### src/main/java/frc/robot/autonomous/AutoRoutines.java

```java
package frc.robot.autonomous;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * The two autonomous routines for the teaching-bot proof of concept.
 *
 * <p>Both are built entirely from commands/DriveDistanceCommand.java and
 * commands/TurnToAngleCommand.java -- no PathPlanner, no vision, just wheel encoders
 * and a gyro. All distances/angles are in feet/degrees, taken straight from
 * {@code Constants.Auto}; {@code DriveDistanceCommand} converts feet to meters
 * internally (see its constructor), so nothing in this file ever touches metric units.
 *
 * <p>{@code final class AutoRoutines} with a {@code private AutoRoutines() {}}
 * constructor and only {@code static} methods is Java's usual stand-in for a
 * module of free functions. Java requires every method to
 * live inside some class, so a class that is never instantiated (only ever referenced
 * as {@code AutoRoutines.driveForwardOnly(...)}) is the idiomatic way to group a small
 * set of related, state-free functions.
 */
public final class AutoRoutines {
    private AutoRoutines() {}

    /** Drives straight forward {@code Auto.DRIVE_FORWARD_ONLY_FEET} feet, then stops. */
    public static Command driveForwardOnly(DriveTrain drivetrain) {
        return new DriveDistanceCommand(drivetrain, DRIVE_FORWARD_ONLY_FEET);
    }

    /**
     * Drives forward, turns, drives forward again:
     * {@code Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET} feet -&gt; turn
     * {@code Auto.DRIVE_TURN_DRIVE_TURN_DEGREES} degrees (positive = left) -&gt;
     * {@code Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET} feet.
     */
    public static Command driveTurnDrive(DriveTrain drivetrain) {
        return Commands.sequence(
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_FIRST_LEG_FEET),
            new TurnToAngleCommand(drivetrain, DRIVE_TURN_DRIVE_TURN_DEGREES),
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_SECOND_LEG_FEET)
        );
    }
}
```

### src/test/java/frc/robot/ElevatorTest.java

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.Elevator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for Elevator's limit-switch-gated raise/lower commands.
 * {@link DIOSim} is
 * keyed by DIO port number, not by reaching into any object, so no visibility changes
 * to Elevator.java were needed to write this file the way DriveTrainTest.java needed
 * one for the SparkMax encoders.
 */
class ElevatorTest {

    private RobotContainer robotContainer;
    private Elevator elevator;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        elevator = robotContainer.elevator;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void raiseCommandStopsAtTop() {
        DIOSim topSim = new DIOSim(Constants.ElevatorConstants.TOP_LIMIT_SWITCH_DIO_PORT);

        topSim.setValue(false); // not at top
        assertFalse(elevator.isAtTop());

        // A command can only be scheduled while the robot is enabled (the default
        // runsWhenDisabled() is false), so enable it first.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        RaiseElevatorCommand command = new RaiseElevatorCommand(elevator);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled()); // whileTrue-style: keeps running while held

        topSim.setValue(true); // reached the top
        step(0.1);
        assertTrue(elevator.isAtTop());
    }

    @Test
    void lowerCommandStopsAtBottom() {
        DIOSim bottomSim = new DIOSim(Constants.ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT);

        bottomSim.setValue(false); // not at bottom
        assertFalse(elevator.isAtBottom());

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        LowerElevatorCommand command = new LowerElevatorCommand(elevator);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        bottomSim.setValue(true); // reached the bottom
        step(0.1);
        assertTrue(elevator.isAtBottom());
    }
}
```

### src/test/java/frc/robot/RobotLifecycleTest.java

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Smoke test: step the whole robot through disabled -&gt; autonomous -&gt; teleop and
 * confirm nothing throws.
 *
 * <p>This file does by hand what a fixture-based
 * test framework would automate: {@link HAL#initialize} boots the simulated hardware layer,
 * {@link DriverStationSim} sets the enabled/autonomous mode flags a real driver station
 * would set, and {@link SimHooks#stepTiming} advances the simulated clock and lets
 * {@link CommandScheduler} run its periodic loop the corresponding number of times.
 * Every test class in this project's {@code src/test/} repeats this same
 * {@code @BeforeEach}/{@code @AfterEach} pair explicitly rather than hiding it behind a
 * shared base class -- consistent with this whole project's "explicit over implicit"
 * rule for anything a rookie might need to step through.
 */
class RobotLifecycleTest {

    private RobotContainer robotContainer;

    @BeforeEach
    void setup() {
        assertDoesNotThrow(() -> {
            if (!HAL.initialize(500, 0)) {
                throw new IllegalStateException("HAL failed to initialize");
            }
        });
        robotContainer = new RobotContainer();
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void stepDisabled(double seconds) {
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    private void stepAutonomous(double seconds) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    private void stepTeleop(double seconds) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void fullModeCycleDoesNotThrow() {
        assertDoesNotThrow(() -> {
            stepDisabled(0.1);
            var autoCommand = robotContainer.getAutonomousCommand();
            if (autoCommand != null) {
                autoCommand.schedule();
            }
            stepAutonomous(1.0);
            stepDisabled(0.1);
            stepTeleop(1.0);
            stepDisabled(0.1);
        });
    }
}
```

### src/test/java/frc/robot/TriggerTest.java

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for Trigger's FireCommand edge-detection state machine.
 *
 * <p>The cam has only one sensor (a limit switch at "home"), so "one fire" is defined
 * as: leave home, then come back to home. These tests exercise that logic directly
 * against the DigitalInput simulation, without needing a real cam mechanism.
 * FireCommand itself has no built-in timeout (see its docstring) -- the second test
 * below applies the same {@code .withTimeout()} decorator RobotContainer.java binds it
 * with, to prove the safety timeout actually works.
 */
class TriggerTest {

    private RobotContainer robotContainer;
    private Trigger trigger;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        trigger = robotContainer.trigger;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void fireCommandFinishesWhenCamReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        // Start "at home" (limit switch reads true, not inverted).
        limitSwitchSim.setValue(true);
        assertTrue(trigger.isAtHome());

        // A command can only be scheduled while the robot is enabled (the default
        // runsWhenDisabled() is false), so enable it first.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        FireCommand command = new FireCommand(trigger);
        command.schedule();
        step(0.1);
        // Still "at home" on the very first tick -- command must not report finished
        // until it has actually left home at least once.
        assertTrue(command.isScheduled());

        // Simulate the cam leaving home.
        limitSwitchSim.setValue(false);
        step(0.1);
        assertTrue(command.isScheduled());

        // Simulate the cam returning home -- command should finish now.
        limitSwitchSim.setValue(true);
        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void fireCommandTimesOutIfNeverReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        limitSwitchSim.setValue(false); // never at home -- simulates a jam
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        // .withTimeout() is the decorator RobotContainer.java actually binds
        // FireCommand with -- applying it here too is what proves the timeout (not
        // just the edge-detection logic) really stops a jammed cam.
        //
        // `.withTimeout(...)` does NOT modify the FireCommand it's called on -- it
        // wraps it in a brand new Command object that composes the original
        // internally. That wrapper is what actually gets scheduled, so it's the
        // wrapper's `isScheduled()` this test has to check below, captured here in a
        // variable typed as the general `Command` interface rather than `FireCommand`
        // (the wrapper is not itself a FireCommand). Checking the original
        // `FireCommand` object's own `isScheduled()` instead would be a real mistake:
        // the scheduler only ever registers the outer wrapper, so the inner
        // FireCommand's `isScheduled()` would incorrectly read `false` from the very
        // first loop, making this test pass without actually exercising the timeout.
        Command timedCommand = new FireCommand(trigger).withTimeout(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS);
        timedCommand.schedule();
        step(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS + 0.5);
        assertFalse(timedCommand.isScheduled());
    }
}
```

### src/test/java/frc/robot/VisionTest.java

```java
package frc.robot;

import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the Vision subsystem's tag-pose lookup and availability reporting.
 *
 * <p>There's no real camera or PhotonVision coprocessor in this test environment, so
 * these tests only cover what doesn't require actual camera data: the AprilTag field
 * layout lookup (a fixed table, not something the camera has to see), and the
 * "nothing has arrived yet" default state. A disconnected {@code PhotonCamera}
 * reports a placeholder result with a near-zero timestamp rather than throwing or
 * returning {@code null}, which is exactly the quirk {@code Vision.periodic()}'s
 * {@code timestamp > 0} check exists to not be fooled by -- see that file for the
 * full explanation. Vision needs no package-private access changes to test, unlike
 * DriveTrain -- everything this test needs is already public.
 */
class VisionTest {

    private RobotContainer robotContainer;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        vision = robotContainer.vision;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    @Test
    void getTagPoseReturnsKnownTag() {
        assertTrue(vision.getTagPose(EXAMPLE_TAG_ID).isPresent());
    }

    @Test
    void getTagPoseReturnsEmptyForUnknownTag() {
        assertTrue(vision.getTagPose(9999).isEmpty());
    }

    @Test
    void noMeasurementWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertTrue(vision.getBestVisionMeasurementIfFresh().isEmpty());
    }

    @Test
    void visionNotAvailableWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertFalse(vision.isAnyVisionAvailable());
    }
}
```

### src/test/java/frc/robot/commands/ApproachTagCommandTest.java

```java
package frc.robot.commands;

import static frc.robot.Constants.METERS_PER_FOOT;
import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for ApproachTagCommand's target computation and 3-phase state machine.
 *
 * <p>This test class lives in {@code frc.robot.commands} (where {@code
 * ApproachTagCommand} itself lives), specifically so it can reach that command's
 * package-private {@code phase}/{@code targetPoint}/{@code turnPid}/{@code drivePid}
 * fields directly -- see the comment on those fields in ApproachTagCommand.java for
 * why they aren't simply {@code private} the way most fields in this project are.
 *
 * <p>This project has no physics simulation wired into its Gradle build (see
 * DriveTrainTest.java's class-level comment), so nothing
 * here overwrites a poked encoder or gyro value on its own -- there's no
 * physics-engine-races-the-scheduler gotcha to work around.
 */
class ApproachTagCommandTest {

    private RobotContainer robotContainer;
    private DriveTrain drivetrain;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.drivetrain;
        vision = robotContainer.vision;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void enable() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void failsForUnknownTag() {
        enable();
        step(0.02);

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, 9999, 3.0);
        command.schedule();
        step(0.02);
        assertFalse(command.isScheduled());
    }

    @Test
    void computesStandoffPoint() {
        Pose2d tagPose = vision.getTagPose(EXAMPLE_TAG_ID).get().toPose2d();

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.initialize();

        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);
        double distanceFromTag = command.targetPoint.getDistance(tagPose.getTranslation());
        assertEquals(3.0 * METERS_PER_FOOT, distanceFromTag, 0.01);
    }

    @Test
    void progressesThroughAllPhases() {
        enable();
        step(0.02);

        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");
        var yawSim = navxSim.getDouble("Yaw");

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.schedule();
        step(0.02);
        assertTrue(command.isScheduled());
        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);

        // Fake having turned exactly to the bearing TURN_TO_TARGET is aiming for.
        // getHeadingDegrees() negates the raw navX yaw (see DriveTrain), so the sign
        // is flipped here to match.
        double bearingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-bearingDegrees);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.DRIVE_TO_TARGET, command.phase);

        // Fake having driven the exact distance this phase is targeting. This test
        // can't reach DriveTrain's package-private leftEncoder/rightEncoder fields
        // directly the way DriveTrainTest.java does -- it lives in
        // frc.robot.commands (to reach ApproachTagCommand's own package-private
        // phase/targetPoint/turnPid/drivePid state above), not
        // frc.robot.subsystems, and package-private access can't span both at
        // once. See setEncoderPositionsForTest()'s doc comment in DriveTrain.java.
        double targetDistance = command.drivePid.getSetpoint();
        drivetrain.setEncoderPositionsForTest(targetDistance, targetDistance);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.FACE_TAG, command.phase);

        // Fake having turned to the final heading -- facing the tag directly in this
        // case, since this command was built with no offset.
        double finalHeadingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-finalHeadingDegrees);
        step(0.02);
        assertFalse(command.isScheduled());
    }
}
```

### src/test/java/frc/robot/subsystems/DriveTrainTest.java

```java
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for DriveTrain's encoder-distance bookkeeping, odometry, kinematics, and
 * its PID autonomous commands.
 *
 * <p>This test class lives in {@code frc.robot.subsystems} (not {@code frc.robot},
 * where most of this project's classes live) specifically so it can reach
 * DriveTrain's package-private {@code leftEncoder}/{@code rightEncoder} fields
 * directly -- see the comment on those fields in DriveTrain.java for why they aren't
 * simply {@code private} the way every other hardware field in this project is.
 *
 * <p>This project has no physics simulation wired into its Gradle build (writing an
 * equivalent {@code simulationPeriodic()} model is a good exercise, not done here) --
 * so nothing here overwrites a poked encoder or gyro value on
 * its own. That keeps these tests simpler:
 * there's no physics-engine-races-the-scheduler gotcha to work around, since nothing
 * is racing. The odometry tests below still call {@code drivetrain.periodic()}
 * directly, even though nothing here
 * strictly requires bypassing the scheduler.
 */
class DriveTrainTest {

    private RobotContainer robotContainer;
    private DriveTrain drivetrain;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.drivetrain;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void enable() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void averageDistanceStartsAtZero() {
        assertEquals(0.0, drivetrain.getAverageDistanceMeters());
    }

    @Test
    void resetEncodersZeroesDistance() {
        drivetrain.resetEncoders();
        assertEquals(0.0, drivetrain.getLeftDistanceMeters());
        assertEquals(0.0, drivetrain.getRightDistanceMeters());
    }

    @Test
    void driveDistanceCommandFinishesOnceTargetReached() {
        // A command can only be scheduled while the robot is enabled (the default
        // runsWhenDisabled() is false), so enable it first.
        enable();
        step(0.02);

        DriveDistanceCommand command = new DriveDistanceCommand(drivetrain, 1.0); // 1 foot
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled()); // nowhere near the target yet

        // Simulate the robot having driven all the way there by writing the target
        // distance straight onto both encoders -- same RelativeEncoder.setPosition()
        // call configureMotors() itself uses to zero them at startup, just called with
        // a nonzero value here.
        double targetMeters = 1.0 * Constants.METERS_PER_FOOT;
        drivetrain.leftEncoder.setPosition(targetMeters);
        drivetrain.rightEncoder.setPosition(targetMeters);

        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void turnToAngleCommandFinishesOnceHeadingReached() {
        enable();
        step(0.02);

        // navX simulation is reached differently from the SparkMax encoders above:
        // Studica's AHRS exposes its simulated yaw through WPILib's generic
        // SimDeviceSim registry under the name "navX-Sensor[4]" rather than through a
        // method on the AHRS object itself -- the same mechanism (and the same
        // device name) the real competition port's own physics simulation uses.
        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");

        TurnToAngleCommand command = new TurnToAngleCommand(drivetrain, 90.0);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        // getHeadingDegrees() negates the raw navX yaw (see DriveTrain), so -90 raw
        // yaw simulates having reached +90 degrees heading.
        navxSim.getDouble("Yaw").set(-90.0);
        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void autoRoutinesBuildWithoutError() {
        assertNotNull(AutoRoutines.driveForwardOnly(drivetrain));
        assertNotNull(AutoRoutines.driveTurnDrive(drivetrain));
        assertTrue(Constants.Auto.DRIVE_FORWARD_ONLY_FEET > 0);
    }

    @Test
    void poseStartsAtOrigin() {
        Pose2d pose = drivetrain.getPose();
        assertEquals(0.0, pose.getX());
        assertEquals(0.0, pose.getY());
        assertEquals(0.0, pose.getRotation().getDegrees());
    }

    @Test
    void odometryTracksStraightLineDriving() {
        // Poke both encoders to a known distance, then call periodic() directly.
        // Heading is left at 0, so odometry should report having moved straight down
        // the field's X axis by exactly this distance.
        drivetrain.leftEncoder.setPosition(2.0);
        drivetrain.rightEncoder.setPosition(2.0);
        drivetrain.periodic();

        Pose2d pose = drivetrain.getPose();
        assertEquals(2.0, pose.getX(), 0.01);
        assertEquals(0.0, pose.getY(), 0.01);
    }

    @Test
    void resetPoseSeedsOdometryAndZeroesEncoders() {
        Pose2d seededPose = new Pose2d(5.0, 1.0, Rotation2d.fromDegrees(90));
        drivetrain.resetPose(seededPose);

        assertEquals(0.0, drivetrain.getLeftDistanceMeters());
        assertEquals(0.0, drivetrain.getRightDistanceMeters());
        Pose2d pose = drivetrain.getPose();
        assertEquals(5.0, pose.getX(), 0.01);
        assertEquals(1.0, pose.getY(), 0.01);
        assertEquals(90.0, pose.getRotation().getDegrees(), 0.5);
    }

    @Test
    void chassisSpeedsZeroWhenStopped() {
        var speeds = drivetrain.getChassisSpeeds();
        assertEquals(0.0, speeds.vxMetersPerSecond, 1e-6);
        assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-6);
    }

    @Test
    void driveDistanceCommandDoesNotCorruptPoseBetweenLegs() {
        DriveDistanceCommand firstLeg = new DriveDistanceCommand(drivetrain, 1.0); // 1 foot
        firstLeg.initialize();

        // Simulate having actually driven 1 foot for this leg.
        double drivenMeters = 1.0 * Constants.METERS_PER_FOOT;
        drivetrain.leftEncoder.setPosition(drivenMeters);
        drivetrain.rightEncoder.setPosition(drivenMeters);
        drivetrain.periodic();

        Pose2d poseAfterLegOne = drivetrain.getPose();
        assertEquals(drivenMeters, poseAfterLegOne.getX(), 0.001);

        // Starting a second DriveDistanceCommand -- exactly what
        // AutoRoutines.driveTurnDrive() does for its second leg -- used to call
        // drivetrain.resetEncoders(), which snapped the tracked pose back toward
        // the origin (see DriveDistanceCommand.initialize()'s doc comment for why).
        // It should now leave the pose exactly where it was.
        DriveDistanceCommand secondLeg = new DriveDistanceCommand(drivetrain, 1.0);
        secondLeg.initialize();
        drivetrain.periodic();

        Pose2d poseAfterSecondLegStarts = drivetrain.getPose();
        assertEquals(drivenMeters, poseAfterSecondLegStarts.getX(), 0.001);
    }
}
```

## Using this as a teaching curriculum

Same reading order as `teaching-bot-odometry-java`, with two files added at the end:

7. **`subsystems/Vision.java`** + **`commands/ApproachTagCommand.java`** -- the
   newest material, and the first genuinely cross-subsystem command in the codebase.
   Read this only after `DriveTrain.java`, since `ApproachTagCommand` builds directly
   on `DriveTrain`'s pose estimate and its existing PID gains. Good discussion
   questions: why does `ApproachTagCommand` call `addRequirements(drivetrain)` but
   not `vision`? Why is it one parameterized class instead of two? What happens if
   the tag ID passed in doesn't exist on the field? And, specific to this branch:
   why couldn't `DriveTrainTest.java`'s package-private trick be reused as-is for
   `ApproachTagCommandTest.java` -- what's actually different about the two
   situations?
8. **Notice that `ApproachTagCommand` never uses `+`/`-` directly on a
   `Translation2d`.** Java has no operator overloading, so every geometric
   combination goes through an explicit method call --
   `Translation2d.plus()`/`.minus()`/`.rotateBy()` -- rather than a symbol. Worth
   pointing out explicitly the first time a rookie hits it here, since it's one of
   the few genuinely new syntactic points this branch introduces that the earlier
   two branches' READMEs didn't already cover.

The value of finishing this branch is seeing how one real bug
(`DriveDistanceCommand` corrupting pose by resetting encoders it no longer owns
exclusively) and one real design tension (one parameterized command vs. two) get
found and resolved once a codebase reaches this point in its own development --
good evidence that these are lessons about robot software design, not lessons tied
to any one project's history.
