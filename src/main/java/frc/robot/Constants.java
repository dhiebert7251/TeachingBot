package frc.robot;

/**
 * Robot-wide constants for the teaching-bot proof of concept. One nested class per
 * subsystem; nothing functional lives here.
 */
public final class Constants {

    private Constants() {}

    // Single conversion point between WPILib's meters and human-facing feet;
    // converted exactly once at the boundary, in DriveDistanceCommand and
    // DriveTrain.periodic().
    public static final double METERS_PER_FOOT = 0.3048;

    public static final class OperatorConstants {
        private OperatorConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveTrainConstants {
        private DriveTrainConstants() {}

        // 4x NEO 2.0 via REV SparkMax, 2 per side (lead + follower).
        public static final int LEFT_LEAD_CAN_ID = 20;
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        public static final int RIGHT_LEAD_CAN_ID = 22;
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        public static final int CURRENT_LIMIT = 60;

        // Drop-center: the center wheel sits 1/4" lower, so each side contacts the
        // ground at only 2 points at once, cutting turn scrub while keeping 6-wheel
        // traction.
        public static final double DROP_CENTER_WHEEL_DROP_METERS = 0.25 * 0.0254; // 1/4 inch, informational only

        public static final double WHEEL_DIAMETER_METERS = 6.0 * 0.0254;
        public static final double WHEEL_WIDTH_METERS = 1.0 * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double GEAR_RATIO = 8.4;

        public static final double TRACK_WIDTH_METERS = 23.0 * 0.0254;
        public static final double WHEEL_CENTER_SPACING_METERS = 13.0 * 0.0254;

        public static final double ROBOT_LENGTH_METERS = 32.0 * 0.0254;
        public static final double ROBOT_WIDTH_METERS = 28.0 * 0.0254;
        public static final double ROBOT_MASS_KG = 118.0 * 0.45359237;

        public static final double JOYSTICK_DEADBAND = 0.05;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;
        public static final double SPEED_SCALE = 0.7;

        public static final double TURN_KP = 0.04;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.005;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;

        // KP is deliberately conservative (a large error would otherwise demand full
        // power); MAX_OUTPUT clamps output as a second, independent safety margin.
        public static final double DRIVE_DISTANCE_KP = 1.5; // TODO: tune on the real robot -- starting point only
        public static final double DRIVE_DISTANCE_KI = 0.0;
        public static final double DRIVE_DISTANCE_KD = 0.1;
        public static final double DRIVE_DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double DRIVE_DISTANCE_MAX_OUTPUT = 0.6; // clamp: never command more than 60% power
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

        // CAN ID: 30s decade, shared with Trigger (same game-piece path).
        public static final int FLYWHEEL_MOTOR_ID = 30;

        public static final boolean FLYWHEEL_INVERTED = false; // TODO: verify on bench

        public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: confirm -- assumed direct-drive until measured
        public static final double SHOOTER_WHEEL_DIAMETER_METERS = 4.0 * 0.0254;

        public static final int CURRENT_LIMIT = 40;

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
        public static final int CAM_CURRENT_LIMIT = 20;
        public static final double CAM_FIRE_SPEED = 0.6;

        public static final int LIMIT_SWITCH_DIO_PORT = 0;
        public static final int BEAM_BREAK_1_DIO_PORT = 1; // ball loaded, waiting to fire
        public static final int BEAM_BREAK_2_DIO_PORT = 2; // ball at the shooter, ready to fire

        public static final boolean LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench (NC vs NO)
        public static final boolean BEAM_BREAK_1_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BEAM_BREAK_2_INVERTED = false; // TODO: verify polarity on bench

        // Safety timeout in case the limit switch never re-triggers (a jam); applied
        // via .withTimeout() where FireCommand is bound in RobotContainer, not inside
        // the command itself.
        public static final double FIRE_TIMEOUT_SECONDS = 2.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        // CAN ID: 40s decade -- a new subsystem family, one decade past Shooter/Trigger.
        public static final int LIFT_MOTOR_ID = 40;

        public static final boolean LIFT_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int LIFT_CURRENT_LIMIT = 30; // Redline motors are small, keep this conservative

        // No encoder on this motor (brushed Redline, no built-in or external sensor)
        // -- this subsystem is entirely open-loop, driven only by limit switches.
        public static final double RAISE_SPEED = 0.5; // spring-assisted, needs less power
        public static final double LOWER_SPEED = -0.7; // against spring tension, needs more power

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
        public static final int ROLLER_CURRENT_LIMIT = 20;

        public static final double INTAKE_SPEED = 1.0;
        public static final double EJECT_SPEED = -1.0;
    }

    public static final class Auto {
        private Auto() {}

        // Feet/degrees (human units); converted to meters only inside
        // DriveDistanceCommand, at the WPILib-math boundary.
        public static final double DRIVE_FORWARD_ONLY_FEET = 10.0;
        public static final double DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0;
        public static final double DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0; // positive = left (CCW)
        public static final double DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0;
    }
}
