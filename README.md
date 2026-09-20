# Teaching-Bot (Java): Proof of Concept

This is a from-scratch WPILib/GradleRIO teaching implementation of a simple FRC
robot, built to a deliberately consistent set of design rules: every command as its
own explicit class with no lambdas, a fixed physical robot, fixed controller
bindings, fixed autonomous routines, and TODO-marked unverified constants throughout.
Each design decision below is documented on its own terms, as a Java codebase.

**Read this before anything else: this project has not been compiled or run.** See
[Verification status](#verification-status) below for exactly why, and exactly what
that does and doesn't mean for trusting this code.

## Contents

- [Verification status](#verification-status)
- [Physical specs](#physical-specs)
- [File structure](#file-structure)
- [Where do commands live?](#where-do-commands-live)
- [Naming and numbering conventions](#naming-and-numbering-conventions)
- [Java language notes](#java-language-notes)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Controller bindings](#controller-bindings)
- [Autonomous](#autonomous)
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
  - Subsystems
    - [DriveTrain.java](#srcmainjavafrcrobotsubsystemsdrivetrainjava)
    - [Elevator.java](#srcmainjavafrcrobotsubsystemselevatorjava)
    - [Gripper.java](#srcmainjavafrcrobotsubsystemsgripperjava)
    - [Shooter.java](#srcmainjavafrcrobotsubsystemsshooterjava)
    - [Trigger.java](#srcmainjavafrcrobotsubsystemstriggerjava)
  - Commands
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
    - [DriveTrainTest.java](#srctestjavafrcrobotsubsystemsdrivetraintestjava)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Verification status

**This code has never been compiled, run, or tested.** It was written in a sandboxed
session whose network policy blocks every Maven repository a WPILib Java project
needs to even download its own dependencies (`frcmaven.wpi.edu`, REVLib's, CTRE's,
and Studica's Maven hosts all returned policy-denied 403s when a build was attempted)
-- not a code problem, a network policy one, confirmed directly before any of this
project's source was written. There was no way to run `./gradlew build` or
`./gradlew test` and see real output.

**A dedicated post-hoc code-review pass (separate from the session that originally
wrote this code) found and fixed one build-breaking bug:** `Robot.java` used to
extend a class called `TimedCommandRobot`, imported from
`edu.wpi.first.wpilibj2.command`. That class does not exist in Java WPILib at all --
there's no Java robot base class that automatically calls
`CommandScheduler.getInstance().run()` every loop; Java's `TimedRobot` requires an
explicit `robotPeriodic()` override to do that. This would have failed to
compile with "cannot find symbol," and even patched to compile, nothing would have
called the scheduler at all -- every command in this project would have been
constructed and bound to a button, and then simply never run. Confirmed by comparing
against the real competition port's own `Robot.java`, which extends AdvantageKit's
`LoggedRobot` (itself a `TimedRobot` subclass) and explicitly overrides
`robotPeriodic()` to call the scheduler. Fixed: `Robot.java` now extends `TimedRobot`
directly, with an explicit `robotPeriodic()` override, matching every real WPILib
Java command-based robot. This is exactly the kind of error a compile step would
have caught in one second and a read-through review can miss -- it's named here
explicitly, not smoothed over, because the whole point of this section is telling you
what has and hasn't actually been checked.

What that means concretely:

- Every WPILib/REVLib/Phoenix6/Studica class name, method name, and argument order
  used below was cross-checked against the REAL competition port
  (`2026_competition_code`, this team's actual 2026 Java robot code) rather than
  written from memory -- `DriveTrain.java` and `Shooter.java` in particular were read
  directly from that repo first, specifically to get REVLib's `SparkMaxConfig`
  pattern and Phoenix 6's `TalonFXConfiguration` pattern exactly right. Where
  something couldn't be cross-checked that way (see below), it's called out
  explicitly rather than presented as equally certain.
- One specific gap: the JUnit tests poke REVLib's SparkMax encoder simulation
  directly via `RelativeEncoder.setPosition(...)`, the same real, documented method
  `DriveTrain`'s own constructor already uses to zero the encoders at startup -- not
  a guessed simulation backdoor. An earlier draft of this file's tests instead
  guessed at a `SimDeviceSim` device name/key for REVLib's simulated encoder, which
  could not be verified in this session and has been removed in favor of that safer,
  confirmed approach (see [Running tests](#running-tests) for the small, deliberate
  visibility change that required).
- **Before trusting this codebase, run
  `./gradlew build test` yourself** (see
  [Building and running this project](#building-and-running-this-project) for the one
  known gap -- a missing binary wrapper jar -- and how to fix it in one command) and
  treat any compile error you find as a real bug report, not a surprise. This is
  exactly the kind of code an independent build should verify before anyone relies on
  it -- the `Robot.java` bug above is proof that a careful reading pass alone,
  without an actual compiler, will miss things.

## Physical specs

The physical robot this code models, on paper:

- **Drivetrain:** 6-wheel "drop center" differential (tank) drive -- 3 wheels per
  side, the center wheel mounted 1/4" LOWER than the front/back wheels, so only the
  center wheel plus one end (front or back) actually touches the ground at a time,
  not all three. That shortens the effective ground-contact wheelbase and reduces
  turning friction compared to a 6-wheel-flat drivetrain, while keeping 6-wheel
  traction/durability for driving straight. 2x NEO 2.0 motors per side (4 total)
  through REV SparkMax controllers, 8.4:1 gearing. 6" diameter, 1"-wide wheels; 13"
  between wheel centers front-to-back per side; 23" between the left and right wheel
  centerlines.
- **Frame:** 32" front-to-back, 28" side-to-side, 118 lb with bumpers.
- **Shooter:** single 5 lb flywheel, 4x 4" compliant wheels as the shooting surface,
  driven by one Kraken X60 (TalonFX) motor.
- **Trigger:** small NEO-driven cam that flicks a game piece into the shooter. One
  limit switch defines the cam's rest ("home") position; two beam-break sensors
  report loading status upstream.
- **Elevator:** 2-stage single-mast cascade elevator (AndyMark "Elevator in a Box"
  style), spring-assisted extension, rope retraction via a geared Redline motor
  (brushed, no encoder). A motor-driven roller gripper sits at the end.

## File structure

```
teaching-bot-java/
├── build.gradle                 # GradleRIO plugin, WPILib + vendor dependencies, JUnit
├── settings.gradle               # Standard GradleRIO plugin-repository setup
├── gradlew, gradlew.bat           # Gradle wrapper scripts (see Building, below, for the wrapper jar gap)
├── gradle/wrapper/                # gradle-wrapper.properties (pins the Gradle version)
├── vendordeps/                    # REVLib, Studica (navX), Phoenix 6, WPILib-New-Commands
├── .wpilib/wpilib_preferences.json
└── src/
    ├── main/java/frc/robot/
    │   ├── Main.java              # Entry point -- do not modify
    │   ├── Robot.java             # extends TimedRobot, calls the scheduler from robotPeriodic()
    │   ├── RobotContainer.java    # Wires subsystems, bindings, and autonomous together
    │   ├── Constants.java          # All numeric/boolean constants, one nested class per subsystem
    │   ├── subsystems/
    │   │   ├── DriveTrain.java
    │   │   ├── Shooter.java
    │   │   ├── Trigger.java
    │   │   ├── Elevator.java
    │   │   └── Gripper.java
    │   ├── commands/                # Every Command, as an explicit class -- see below
    │   │   ├── TeleopDriveCommand.java, ResetGyroCommand.java, DriveDistanceCommand.java, TurnToAngleCommand.java
    │   │   ├── SpinUpShooterCommand.java
    │   │   ├── FireCommand.java
    │   │   ├── RaiseElevatorCommand.java, LowerElevatorCommand.java
    │   │   └── IntakeCommand.java, EjectCommand.java
    │   └── autonomous/
    │       ├── AutoRoutines.java   # The two autonomous routines (built from commands/)
    │       └── AutoChooser.java    # Builds the SmartDashboard auto chooser
    └── test/java/frc/robot/         # JUnit 5 test suite -- see "Running tests"
```

Java's package system requires a directory layout that mirrors the package
declaration at the top of every file: `package frc.robot.subsystems;` means the file
must live in `.../frc/robot/subsystems/`. That's a compiler requirement, not a style
choice -- the directory tree above is exactly as deep as the package names it holds.

## Where do commands live?

Every command in this project -- teleop driving, both autonomous PID commands, firing the trigger,
raising/lowering the elevator, running the gripper -- is its own explicit class in
`commands/`, subclassing `Command` and overriding whichever of `initialize()`/
`execute()`/`isFinished()`/`end(boolean interrupted)` it actually needs. Subsystems
only expose plain, one-shot hardware actions (`drive()`, `stop()`,
`getLeftDistanceMeters()`, ...) -- nothing that returns a `Command`.

The alternative this project deliberately did NOT take -- and the one most WPILib
Java example code (including this team's own real competition port) actually uses --
is a **factory-method** style: a subsystem method like
`Command driveDistanceCommand(double feet)` that builds and returns a command in one
expression using `Commands.run(...)`, `.until(...)`, `.finallyDo(...)`, and similar
composition helpers. Compare this project's `commands/FireCommand.java` to what it
would look like as a factory method instead:

```java
// The factory-method version this replaced -- functionally identical, but its one
// bit of state has to be captured by a lambda closing over a local variable
// (Java requires that captured variable to be "effectively final," which rules out
// a plain local boolean the lambda could reassign -- an AtomicBoolean or a one-
// element array is the usual workaround), and "when does this run" is implicit in
// how .until()/.finallyDo() are composed rather than four named methods.
public Command fireCommand() {
    var hasLeftHome = new java.util.concurrent.atomic.AtomicBoolean(false);
    return Commands.runOnce(() -> hasLeftHome.set(false), this)
        .andThen(Commands.run(this::runCam, this)
            .until(() -> {
                if (!hasLeftHome.get()) {
                    if (!isAtHome()) {
                        hasLeftHome.set(true);
                    }
                    return false;
                }
                return isAtHome();
            }))
        .finallyDo(interrupted -> stopCam());
}
```

Both versions do the same thing. The explicit-class version needs no
`AtomicBoolean` workaround for shared mutable state (a plain `private boolean` field
is enough, since it's a real field on a real object, not a variable captured by a
closure), its lifecycle methods are separately readable, and each one is a plain
method that can be stepped through in a debugger without also having to understand
lambda captures and command-decorator composition at the same time. The cost is more
boilerplate per command (a constructor and an `addRequirements()` call every class
needs). For a rookie-facing codebase, that trade is worth it: readability for
whoever steps through this code later matters more than the lines saved by
composition helpers, especially since Java's "effectively final" capture rule makes
the lambda version's shared mutable state noticeably awkward to work with.

**No lambdas or inline commands anywhere in this project, including one-shot
actions.** `ResetGyroCommand.java` is a full class whose `initialize()` does the
work and whose `isFinished()` returns `true` immediately, rather than the shorter
(but lambda-based) `Commands.runOnce(drivetrain::resetGyro, drivetrain)`.

## Naming and numbering conventions

Standard FRC-wide and team conventions, matching the real competition port (CAN IDs
grouped by subsystem in tens, constants in one file/class, one
file/class per subsystem, `<Verb><Noun>Command` naming, SI units internally with
imperial only at input/output boundaries, positive rotation = counterclockwise). Two
conventions below are specific to Java as a language:

- **Java naming convention:** classes `PascalCase`, methods and
  fields `camelCase`, constants `ALL_CAPS_WITH_UNDERSCORES`,
  packages all-lowercase (`frc.robot.subsystems`). This convention is followed
  consistently everywhere in the codebase, including generated-looking getters like
  `getLeftDistanceMeters()`.
- **No "vendor library uses different casing" seam.** This project's own code and
  every vendor library's code (REVLib, Phoenix 6, WPILib itself) use the exact same
  camelCase convention, because Java has only ever
  had one dominant naming convention across its whole ecosystem. `phoenix6`'s Java
  API is `com.ctre.phoenix6.hardware.TalonFX`, same camelCase as everything else.
  Nothing to teach here beyond "Java is camelCase, always."

## Java language notes

A few syntax rules Java enforces that are easy to gloss over when skimming the code
quickly. See each command class's own comments for the fullest version of this
(starting with `commands/TeleopDriveCommand.java`'s constructor, which walks through
every piece individually), but the short version:

| Concept | Java rule |
|---|---|
| Type on a parameter/field | Required, before the name: `DriveTrain drivetrain` -- checked by the compiler at every call site. |
| "Private" | The `private` keyword, compiler-enforced -- there is no way to reach a private member from outside its declaring class. |
| A method returning nothing | Declared `void methodName()` explicitly -- no return type can be omitted. |
| A constructor | Has no return type at all -- not even `void` is legal syntax for one. |
| Calling the parent constructor | Automatic if omitted: Java always calls the no-arg parent constructor first unless a class explicitly calls a different one. |
| The current object | `this` -- often omittable, and required only when a field and a parameter share a name. |
| A file of related functions with no class | Not possible -- every method must be `static` on some class (see `AutoRoutines.java`, whose static methods play that role). |
| A "constant" | `public static final TYPE NAME = value;` -- three keywords doing three separate jobs: `static` (one shared copy), `final` (assignable once), and the type. |

## Subsystems

| Subsystem | File | Purpose | Hardware | Key methods |
|---|---|---|---|---|
| **DriveTrain** | `subsystems/DriveTrain.java` | Moves the robot (tank/differential drive) | 4x REV SparkMax NEO 2.0 (CAN 20-23), navX2 gyro (SPI/MXP) | `drive()`, `stop()`, `getLeft/RightDistanceMeters()`, `getLeft/RightVelocityMetersPerSecond()`, `getHeadingDegrees()`, `resetGyro()`, `resetEncoders()` |
| **Shooter** | `subsystems/Shooter.java` | Spins the flywheel to a fixed target RPM | 1x Kraken/TalonFX (CAN 30) | `setTargetRpm()`, `stop()`, `getCurrentRpm()`, `isAtTargetSpeed()` |
| **Trigger** | `subsystems/Trigger.java` | Fires one game piece into the shooter per cam revolution | 1x NEO/SparkMax (CAN 31), 1 limit switch (DIO 0), 2 beam breaks (DIO 1-2) | `runCam()`, `stopCam()`, `isAtHome()`, `hasBallAtStage1()`, `hasBallAtStage2()` |
| **Elevator** | `subsystems/Elevator.java` | Raises/lowers the 2-stage mast | 1x geared Redline/SparkMax, brushed, no encoder (CAN 40), top/bottom limit switches (DIO 3-4) | `setSpeed()`, `stop()`, `isAtTop()`, `isAtBottom()` |
| **Gripper** | `subsystems/Gripper.java` | Spinning roller intake at the end of the elevator | 1x NEO 550/SparkMax (CAN 41) | `setSpeed()`, `stop()` |

Notice none of these have a method that returns a `Command` -- that's the point of
the split described in [Where do commands live?](#where-do-commands-live).

## Commands

| Command | Subsystem | Bound as | What it does |
|---|---|---|---|
| `TeleopDriveCommand` | DriveTrain | Default command | Tank drive read directly from the driver controller's two joystick Y-axes |
| `ResetGyroCommand` | DriveTrain | `onTrue` | One-shot: resets the navX heading to 0 |
| `DriveDistanceCommand(feet)` | DriveTrain | Used in autonomous | PID-drives straight to a target distance, in feet |
| `TurnToAngleCommand(degrees)` | DriveTrain | Used in autonomous | PID-turns to an absolute heading using the navX gyro |
| `SpinUpShooterCommand` | Shooter | `toggleOnTrue` | Toggle: commands the flywheel to a fixed target RPM, or stops it |
| `FireCommand` | Trigger | `onTrue`, with `.withTimeout(...)` | Runs the cam motor until it leaves and then returns to the home (limit-switch) position |
| `RaiseElevatorCommand` / `LowerElevatorCommand` | Elevator | `whileTrue` | Drives the lift motor while held, auto-stopping at the relevant limit switch |
| `IntakeCommand` / `EjectCommand` | Gripper | `whileTrue` | Spins the roller in/out while held |

## Controller bindings

**Driver (port 0) -- drive only:**

| Input | Action |
|---|---|
| Left stick Y | Left-side drive speed |
| Right stick Y | Right-side drive speed |
| Back button | Reset gyro heading to 0 (do this before every autonomous run) |

**Operator (port 1) -- everything else:**

| Input | Action |
|---|---|
| A | Toggle shooter spin-up |
| B | Fire trigger |
| X | Gripper intake (while held) |
| Y | Gripper eject (while held) |
| Right Bumper | Raise elevator (while held) |
| Left Bumper | Lower elevator (while held) |

## Autonomous

| Option | What it does |
|---|---|
| **Do Nothing** (default) | `Commands.none()` -- the robot does nothing during autonomous |
| **Drive Forward 10 ft** | `AutoRoutines.driveForwardOnly(drivetrain)`, straight |
| **Drive 5ft, Turn Left 90, Drive 3ft** | `AutoRoutines.driveTurnDrive(drivetrain)` -- a `Commands.sequence(...)` of the two PID commands with a turn in between |

## Running tests

```
./gradlew test
```

Five files in `src/test/java/frc/robot/`:

| Java | Covers |
|---|---|
| `RobotLifecycleTest.java` | Full disabled → autonomous → teleop mode cycle, no exceptions |
| `subsystems/DriveTrainTest.java` | Encoder bookkeeping, both PID commands reaching their setpoint, autonomous routines building without error |
| `ElevatorTest.java` | Limit-switch-gated raise/lower commands |
| `TriggerTest.java` | `FireCommand`'s edge-detection state machine, including its timeout |

**GradleRIO ships no ready-made simulation fixture.** Every Java test file here sets
up the simulated hardware layer itself, explicitly, in its own
`@BeforeEach`/`@AfterEach`, using `HAL.initialize()`
to boot the simulated hardware layer, `DriverStationSim` to set the enabled/autonomous
flags a real driver station would set, and `SimHooks.stepTiming()` to advance
simulated time and run the CommandScheduler's periodic loop -- consistent with this
whole project's "explicit over implicit" rule.

**A real design change worth reading as its own lesson:**
`subsystems/DriveTrain.java`'s `leftEncoder`/`rightEncoder` fields are
**package-private** (no access modifier at all), not `private` like every other
hardware field in this project. `DriveTrainTest.java` needs to poke their simulated
position directly with `.setPosition(...)` to test `DriveDistanceCommand`/
`TurnToAngleCommand` without a real robot. Java's `private`
is compiler-enforced with no loophole a test could reach around, so getting that
access for real needed an actual, coarser access level -- package-private: visible
to any class in `frc.robot.subsystems`, invisible everywhere else -- rather than
loosening the field all the way to `public`. Worth sitting with: here, the test's
needs directly shaped a production access modifier, not just the test code itself.

## Building and running this project

```
./gradlew build   # compile + run tests
./gradlew simulateJava   # run in WPILib's desktop simulator
```

**One known gap:** `gradle/wrapper/gradle-wrapper.jar` -- the small binary file that
makes `./gradlew` work without a separately-installed Gradle -- is **not included**
in this branch. It's a compiled binary, and the tooling used to write this project
could only commit text files. Before running `./gradlew` for the first time, generate
it with either:

```
gradle wrapper --gradle-version 8.11   # if you have Gradle installed some other way
```

or open this folder in VS Code with the WPILib extension installed and let it manage
the wrapper for you (the extension bundles its own Gradle/JDK and doesn't strictly
need `./gradlew` to work), or copy `gradle/wrapper/gradle-wrapper.jar` from any other
2026 GradleRIO project you already have checked out (this team's own
`2026_competition_code` repo has one).

## Design decisions and deliberate simplifications

Deliberate simplifications made throughout this codebase:

- Every command is an explicit class, not a factory method (see
  [Where do commands live?](#where-do-commands-live) above for the Java-specific
  side-by-side comparison).
- `DriveDistanceCommand` is PID, not "drive at a fixed speed until the encoder says
  stop," with its output independently clamped as a safety margin on top of a
  conservative `KP`.
- Elevator is entirely open-loop -- no encoder exists to close a position loop
  against, only two limit switches.
- Trigger's "fire" is an edge-detection state machine, not a single sensor read, since
  the cam starts every cycle already at its one "home" sensor.
- Every command has the same four-method shape, but not every command needs all four
  -- `SpinUpShooterCommand` has no `execute()` because Phoenix 6's velocity control is
  closed-loop on the TalonFX itself.

## Assumptions that need bench verification

TODO-marked constants, all in `Constants.java`
-- motor inversions, limit-switch/beam-break polarity, every PID gain, the shooter's
gear ratio. None of these are guesses about whether the CODE is correct; they're
guesses about the ROBOT this code hasn't met yet.

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
// Example: import com.revrobotics.spark.SparkMax;
// Example: import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: one-sentence description of the real-world mechanism this subsystem
 * controls (what hardware it owns, and in one clause, why it exists).
 * Example: "Shooter subsystem -- single flywheel motor, no sensors."
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
    // Example: private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);

    /**
     * Constructor: construct and configure every hardware object this subsystem
     * owns. Motor inversions, current limits, idle modes, and encoder conversion
     * factors belong here -- configured once, at startup, not repeated every
     * loop in periodic() below.
     */
    public BlankSubsystem() {
        // TODO: construct hardware objects and configure them.
        // Example: leftMotor.setIdleMode(IdleMode.kBrake);
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
        // Example: SmartDashboard.putNumber("BlankSubsystem/SpeedCommanded", leftMotor.get());
    }

    // Public methods: everything a Command needs in order to actually use this
    // subsystem goes here, as small, mechanical methods -- e.g. setSpeed(double),
    // isAtSetpoint(), a getter for the latest sensor reading. Keep the decision
    // of WHEN to call them out of this class; that belongs in a Command.
    // Example:
    // public void setSpeed(double speed) {
    //     leftMotor.set(speed);
    // }
}
```

### BlankCommand.java (template)

```java
package frc.robot.commands;

// Imports: WPILib's Command base class, plus whichever subsystem(s) this command
// requires and any math/utility classes its logic needs (a PIDController, for
// example -- see DriveDistanceCommand.java for a real one).
// Example: import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlankSubsystem;

/**
 * TODO: one-sentence description of the real-world behavior this command
 * produces once it's scheduled.
 * Example: "Spins the shooter flywheel up to a fixed target speed and holds it there."
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
    // Example: private final PIDController pid = new PIDController(TARGET_KP, TARGET_KI, TARGET_KD);

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
        // Example: pid.reset();
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
        // Example: subsystem.setSpeed(pid.calculate(subsystem.getSpeed()));
    }

    /**
     * isFinished(): checked every loop, right after execute(). Return true the
     * instant this command's job is done; the scheduler then calls end(false)
     * and stops scheduling it. Returning the constant `false` (as written here)
     * makes this a command that never finishes on its own -- correct for
     * something meant to run until interrupted (like TeleopDriveCommand), wrong
     * for anything that should end automatically once a condition is met (like
     * DriveDistanceCommand reaching its target).
     * Example: return pid.atSetpoint();
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
        // Example: subsystem.stop();
    }
}
```

## Annotated source code

The code in this repository has had its comments trimmed to near-zero (see each file); this section preserves the original, fully-annotated teaching version of every file for reference.

### src/main/java/frc/robot/Constants.java

```java
package frc.robot;

/**
 * Robot-wide numerical/boolean constants for the teaching-bot proof of concept.
 *
 * <p>One nested class per subsystem, same convention as the real competition port
 * (2026_competition_code) -- see that repo's Constants.java. Nothing functional lives
 * here, only numbers/IDs.
 *
 * <p>Java has no convention-only notion of a constant: every field
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
 * {@code edu.wpi.first.wpilibj2.command}. That class does not exist in Java WPILib at
 * all -- there's no Java robot base class that automatically calls
 * {@code CommandScheduler.getInstance().run()}
 * every loop; the mistake would have failed to compile with "cannot find symbol," and even patched to
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
 * logging.
 */
public class Robot extends TimedRobot {
    // `Command` (an interface/abstract class) is the TYPE; `m_autonomousCommand` can
    // hold `null` (no autonomous command selected) or any object that implements
    // Command. Java has no separate "nullable" annotation built into the language --
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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoChooser;
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

/**
 * RobotContainer for the teaching-bot proof of concept.
 *
 * <p>Wires the five subsystems together, sets teleop default commands and button
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
    public final DriveTrain drivetrain = new DriveTrain();
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
     * <p>Driver (port 0) -- drive only:
     * <ul>
     *   <li>Left Y / Right Y = tank drive</li>
     *   <li>Back = reset gyro heading to 0 (do this before autonomous!)</li>
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
 * <p>{@code extends SubsystemBase} pulls in the WPILib base
 * class that registers this object with the CommandScheduler and gives it a default,
 * do-nothing {@code periodic()} to override.
 *
 * <p>This subsystem deliberately stops at raw encoder distances and a raw gyro heading
 * -- no PathPlanner, no vision, no pose estimator/odometry fusing them together. Pose
 * estimation is a natural <i>next</i> lesson once encoders, gyro, and PID are all
 * comfortable on their own, not a starting one (see the {@code teaching-bot-odometry-java}
 * branch, which builds on this one).
 */
public class DriveTrain extends SubsystemBase {

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
    // is a real access restriction the compiler checks, not just a naming convention.
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
    // `.setPosition(...)` to test DriveDistanceCommand/TurnToAngleCommand without a
    // real robot. Java's `private` is enforced by the
    // compiler with no loophole a test could reach around, so getting that test access
    // here needs an actual, coarser access level rather than a naming hint. Package-private
    // is the narrowest level that still works: any class in frc.robot.subsystems can
    // reach these fields, but nothing outside that package (including
    // RobotContainer.java, in frc.robot) can -- a real, if slightly wider, restriction,
    // not merely a polite request.
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

    // Used by periodic() below to only publish telemetry every Nth loop instead of
    // every ~20ms -- SmartDashboard/NetworkTables traffic adds up, and nothing reads
    // these values fast enough to need them every single loop. Not `final`: this one
    // field IS reassigned, every loop, in periodic() below.
    private int telemetryLoopCounter = 0;

    /**
     * The constructor -- Java calls this automatically for {@code new DriveTrain()}.
     * Unlike every command class in this project (see commands/TeleopDriveCommand.java
     * for the full explanation of constructor syntax), this constructor takes no
     * parameters at all: DriveTrain doesn't need anything handed to it from the
     * outside to build itself, since every value it needs (CAN IDs, current limits)
     * comes from {@code Constants.DriveTrainConstants} instead.
     */
    public DriveTrain() {
        configureMotors();
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
        // every SparkMax on this robot.
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
    // internal numbers in meters means it can be handed directly to PID code without a
    // conversion at every call site. The conversion to feet (for humans) happens in
    // exactly two places: this file's periodic() telemetry, and
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
     * no colon, for every parameter and every field in this project, and the compiler
     * requires it on every one.
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
 * <p>Phoenix 6's Java enum constant names use their own capitalization
 * ({@code InvertedValue.Clockwise_Positive}) rather than the ALL_CAPS_WITH_UNDERSCORES
 * convention Java enum constants otherwise use everywhere in this project -- one
 * vendor library's own naming choice, not a rule of the language. Worth flagging
 * explicitly the first time a rookie hits it, since it breaks the otherwise-uniform
 * pattern.
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
        // when it's constructed, which for autonomous commands happens once at
        // RobotContainer startup, possibly minutes before the command actually runs).
        // Zeroing the encoders and the PID controller here means "distance driven" is
        // always measured from wherever the robot happens to be right now.
        drivetrain.resetEncoders();
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
        double output = pid.calculate(drivetrain.getAverageDistanceMeters());
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
     *       this on every field and method, checked by the compiler at compile
     *       time.</li>
     *   <li><b>{@code DriveTrain drivetrain}</b> -- a parameter.
     *       Java requires every parameter to have a declared type, written
     *       BEFORE the name with no colon, and the compiler itself refuses to compile
     *       code that passes the wrong type here -- there is no way to skip this
     *       declaration.</li>
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
     *       error, not just bad style. Java's constructor
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
        // same name on purpose (this project's Java convention), which means `this.`
        // in front of the left-hand side is not optional decoration here: without it, `drivetrain
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
 * constructor and only {@code static} methods is the idiomatic Java way to group a
 * small set of related, state-free functions. Java requires every method to
 * live inside some class, so a class that is never instantiated (only ever referenced
 * as {@code AutoRoutines.driveForwardOnly(...)}) stands in for what would otherwise
 * be a bare collection of top-level functions.
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
 * <p>WPILib's
 * Java toolchain ships no ready-made simulation fixture, so this file does the setup
 * by hand: {@link HAL#initialize} boots the simulated hardware layer,
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

### src/test/java/frc/robot/subsystems/DriveTrainTest.java

```java
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
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
 * Unit tests for DriveTrain's encoder-distance bookkeeping and its PID autonomous
 * commands.
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
 * its own, which keeps these tests simple: there's no physics-engine-races-the-scheduler
 * gotcha to work around, since nothing is racing.
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
}
```

## Using this as a teaching curriculum

Suggested reading order: `Constants.java` first, then
`Gripper`/`GripperCommands` as the simplest pair, then `Elevator`, `Trigger`,
`Shooter`, `DriveTrain` last, then `RobotContainer.java`, then `src/test/`.
`commands/TeleopDriveCommand.java`'s constructor is worth lingering on in
particular -- see the "Java language notes" section and its own comments (in the
[Annotated source code](#annotated-source-code) appendix above) for a line-by-line
walkthrough of the syntax rules it demonstrates: type placement, `this`, `private`,
and the constructor return-type rule.
