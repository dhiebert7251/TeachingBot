# Teaching-Bot (Java): AprilTag Vision

This is the **Java sibling** of `teaching-bot-vision` (the Python branch of this same
repo), which itself builds on `teaching-bot-odometry`. This branch builds on
`teaching-bot-odometry-java` the same way: adding one PhotonVision camera doing
AprilTag pose estimation, fusing its fixes into `DriveTrain`'s pose estimator to
correct the dead-reckoning drift the odometry branch's README named as its reason for
existing, and one new cross-subsystem command, `ApproachTagCommand`, built on top of
that. Every other design decision -- explicit-class commands, no lambdas, the same
physical robot -- carries over unchanged from `teaching-bot-poc-java`; see that
branch's README for the full reasoning behind those, and for the general
Java-vs-Python comparison material this README doesn't repeat.

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
- [Annotated source code](#annotated-source-code)
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
per camera. Both patterns exist in PhotonLib for the 2026 season. This file uses the
first pattern because it's the one actually verified end to end in this project's
Python sibling -- constructed, called, and passed against the real installed
`photonlibpy==2026.3.4` package earlier in this same overall effort -- while the
competition port's own pattern, though real production code, was only read, never
run, in this particular Java session. Evidence that something actually executed
successfully once beats evidence that something merely compiles by inspection, which
is why this file mirrors the tested pattern instead of the unverified-here one, even
though the unverified-here one is what this team's own real robot runs. **If
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
  safety gap -- but it is not unique to this Java branch: the already-tested Python
  sibling's own `robotcontainer.py` binds `ApproachTagCommand` the exact same way,
  with no timeout either. It's therefore documented here as a shared, cross-language
  design point rather than patched only on the Java side, which would have silently
  broken the 1:1 comparison this whole project exists to support. See
  [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
  below.

## What changed from teaching-bot-odometry-java

- **`Constants.java`** gained a `VisionConstants` nested class: camera name, the
  robot-to-camera mounting transform, vision quality-gating thresholds, the three
  standard-deviation matrices, and the two example `ApproachTagCommand` bindings'
  numbers (tag ID, standoff distances, turn offset).
- **New file, `VisionMeasurement.java`** -- a `record` bundling one vision pose fix
  (`estimatedPose`, `timestampSeconds`, `standardDeviations`, `numTagsUsed`). See that
  file's own Javadoc for why `record` is the right tool here, and how it compares to
  Python's `@dataclass(frozen=True)`.
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
  is easy to get backwards by guessing. This exact sign convention WAS verified
  directly, but against the Python sibling's installed `wpimath` package, not this
  branch's own Java one (see [Verification status](#verification-status) above for
  why that gap exists in this session): rotating a "straight ahead" translation by
  `Rotation3d(0, pitch, 0)` gave a negative pitch a positive Z component (tilted up).
  Java and Python `wpimath` share the same underlying C++ geometry implementation, so
  this convention should carry over unchanged -- but "should," not "was independently
  confirmed here," is the honest way to describe it. Worth a rookie re-running that
  same one-line experiment in Java once a build environment exists, rather than
  taking this comment on faith -- that's the whole point of writing it down as a
  claim someone can check, instead of a fact.

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
[Verification status](#verification-status) above for why that's a real, if shared
with the Python sibling, gap worth a rookie noticing.

Both are the same command class, parameterized -- **not** two near-duplicate classes
the way `RaiseElevatorCommand`/`LowerElevatorCommand` are. That's a deliberate
exception to this project's usual "each distinct behavior gets its own named class"
convention, made because these two behaviors share the entire 3-phase state machine
below and differ by exactly one number (`faceOffsetDegrees`, defaulted to `0.0` via a
second, shorter constructor overload -- Java has no default-parameter-value syntax
the way Python's `face_offset_degrees: float = 0.0` does, so an overload is the
idiomatic Java equivalent). Two copies of that state machine would mean any future
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
(`driveDistanceCommandDoesNotCorruptPoseBetweenLegs`, a Java translation of the
Python sibling's own regression test for the bug described in
[Design decisions](#design-decisions-and-deliberate-simplifications) below).

**A second package-private-for-testability exception, and why one exception led to
another.** `DriveTrainTest.java` already needed `DriveTrain.leftEncoder`/
`rightEncoder` to be package-private instead of `private` (see
`teaching-bot-poc-java`'s README for the original reasoning) so a same-package test
could poke simulated encoder position directly. `ApproachTagCommand.java` needed the
same kind of access for its own internal `phase`/`targetPoint`/`turnPid`/`drivePid`
fields, so `ApproachTagCommandTest.java` can check the command's state-machine
progress the same way the Python sibling's test reaches its `_phase`/`_target_point`/
`_turn_pid`/`_drive_pid` past Python's naming-convention-only privacy. That test
therefore has to live in `frc.robot.commands` (where `ApproachTagCommand` itself
lives) -- but it *also* needs to poke `DriveTrain`'s encoders to fake a driven
distance mid-test, and package-private access never spans two different packages no
matter how either side is declared: a test can be a member of `frc.robot.commands` or
`frc.robot.subsystems`, never both at once. Rather than force one package-private
trick to do a job it structurally cannot, `DriveTrain` gained one small, honestly
named `public` method, `setEncoderPositionsForTest(double, double)`, whose Javadoc
says exactly what it's for and why it exists. Three real lessons in one small corner
of this codebase: Java's `private` is enforced in a way Python's naming convention
never is; package-private is the standard fix for a same-package test that needs
past that enforcement; and package-private has a hard structural limit -- once a
test's need spans two different production packages, a plain `public` method,
clearly labeled, is the more honest tool than trying to bend visibility rules to fit.

Everything else about running these tests -- no bundled fixture equivalent to
pyfrc's `robot`/`control`, `HAL.initialize()`/`DriverStationSim`/`SimHooks` used
explicitly in every file's `@BeforeEach`, no physics simulation wired into this
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
  This matches the already-tested Python sibling's own bindings exactly, so it's a
  shared design point across both language ports rather than something introduced
  here, but it's a real gap worth fixing (in both projects, together, so the
  comparison stays fair) before this pattern is ever bound on hardware that can
  actually hurt someone. A reasonable fix: wrap both bindings in
  `.withTimeout(...)`, the same decorator `FireCommand`'s binding already uses, with
  a new `VisionConstants` constant for the timeout duration.
- **The `PhotonPoseEstimator` construction pattern itself** (see
  [Verification status](#verification-status) above) -- worth treating as an open
  question to settle, not just a code style preference, once a real build/test
  environment exists: does `estimateCoprocMultiTagPose()`/
  `estimateLowestAmbiguityPose()` actually exist with those exact names in the
  installed `v2026.3.2` PhotonLib Java jar the way they do in `photonlibpy`? If not,
  this file needs to move to the competition port's `PoseStrategy`-based pattern
  instead, and this README's framing of that choice was wrong.
- **`DriveTrain.resetPose()` resets the hardware encoders; the real competition
  port's own `resetPose()` doesn't** (see [Verification status](#verification-status)
  above) -- worth deciding deliberately, not by inertia, whether this teaching
  project's behavior or the competition robot's is the one worth adopting if this
  code is ever used as a template for real robot work.

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
 * <p>A note for anyone coming from the Python sibling of this project
 * (teaching-bot-poc, in this same repo): Python's constants.py used a plain class per
 * subsystem with bare {@code ALL_CAPS = value} attributes -- Python doesn't require a
 * value to be typed, and a class attribute is "constant" purely by convention (nothing
 * stops code from reassigning it). Java has no such convention-only option: every field
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
        // Sign convention verified directly (not assumed) against the Python sibling's
        // installed wpimath package earlier in this project's development: rotating
        // Translation3d(1, 0, 0) -- "straight ahead" -- by Rotation3d(0, pitch, 0) gives
        // a NEGATIVE pitch a positive Z component (tilted up) and a POSITIVE pitch a
        // negative Z component (tilted down). "Angled upward" is therefore a negative
        // pitch here -- worth double-checking against whichever WPILib version is
        // installed if this is ever copied elsewhere, since it's easy to get backwards.
        // wpimath's Java and Python builds share the same underlying C++ geometry
        // implementation, so this convention carries over unchanged, but it was not
        // re-verified against the Java package specifically (see README).
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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Entry point for the teaching-bot proof of concept.
 *
 * <p><b>Corrected during a post-hoc code-review pass:</b> this class used to extend a
 * class called {@code TimedCommandRobot}, imported from
 * {@code edu.wpi.first.wpilibj2.command}. That class does not exist in Java WPILib --
 * it's a RobotPy-only convenience ({@code commands2.TimedCommandRobot}, in the Python
 * bindings) that automatically calls {@code CommandScheduler.getInstance().run()}
 * every loop; there's no Java equivalent that does the same thing implicitly. The
 * mistake would have failed to compile with "cannot find symbol," and even patched to
 * compile, nothing would have called the scheduler at all -- autonomous and teleop
 * commands would never actually run.
 *
 * <p>The fix: extend the real {@link TimedRobot} directly, and call the scheduler
 * explicitly, once, from an overridden {@code robotPeriodic()} below -- exactly what
 * every WPILib Java command-based robot does, including this team's own real
 * competition port (whose {@code Robot.java} extends AdvantageKit's
 * {@code LoggedRobot}, itself a {@code TimedRobot} subclass, and does this same
 * explicit call).
 *
 * <p>No AdvantageKit, no vision-specific logging (unlike the real competition port's
 * {@code Robot.java}) -- just {@link DataLogManager} for on-disk + NetworkTables
 * logging, matching the Python teaching-bot's {@code robot.py} exactly.
 */
public class Robot extends TimedRobot {
    // `Command` (an interface/abstract class) is the TYPE; `m_autonomousCommand` can
    // hold `null` (no autonomous command selected) or any object that implements
    // Command. Python's equivalent used `Optional[Command] = None` as a type hint --
    // Java has no separate "nullable" annotation built into the language the way
    // Python's `Optional[X]` is; ANY non-primitive Java type (anything that isn't
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

    /**
     * Runs every ~20ms, no matter what mode the robot is in -- this is the one place
     * {@code CommandScheduler.getInstance().run()} has to be called from. It's what
     * actually polls button bindings, starts newly-scheduled commands, runs already-
     * scheduled commands' {@code execute()}, checks {@code isFinished()}, and calls
     * every registered subsystem's {@code periodic()}. Without this override, nothing
     * in the command-based framework -- not a single command, not a single
     * subsystem's {@code periodic()} -- would ever run.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
     * buttons to commands for an Xbox-style controller as of WPILib 2026 -- the same
     * class name in both the Java and Python bindings (Python's is a thin wrapper
     * around this very Java/C++ implementation, which is why the class names and
     * method names on it already match almost exactly between the two languages,
     * unlike REVLib/Phoenix6/Studica's separately-written Java and Python APIs).
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
 * {@code hashCode()}/{@code toString()} -- without writing any of that by hand. It's
 * the closest Java equivalent to Python's {@code @dataclass(frozen=True)}, used for
 * the exact same purpose on the Python sibling's own {@code VisionMeasurement}. This
 * project's real competition port ({@code 2026_competition_code}) uses this same
 * {@code record} pattern for its own {@code VisionMeasurement} class, accessed the
 * same parenthesized way ({@code measurement.estimatedPose()},
 * {@code measurement.numTagsUsed()}) -- that usage is where this file's shape was
 * confirmed from, rather than guessed.
 *
 * <p>{@code standardDeviations} is a {@code Matrix<N3, N1>} (a 3-row, 1-column
 * matrix) rather than a plain 3-tuple the way the Python sibling wrote it -- Java's
 * {@code DifferentialDrivePoseEstimator.addVisionMeasurement(...)} takes its standard
 * deviations as this typed matrix, built with {@code VecBuilder.fill(x, y, theta)};
 * RobotPy's Python binding for the same underlying method instead accepts a plain
 * tuple, which is why the two sibling projects' constants differ in shape here even
 * though they encode the exact same three numbers.
 */
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed
) {}
```

