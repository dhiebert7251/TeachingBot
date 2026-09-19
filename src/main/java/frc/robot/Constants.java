package frc.robot;

public final class Constants {

    private Constants() {}

    public static final double METERS_PER_FOOT = 0.3048;

    public static final class OperatorConstants {
        private OperatorConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveTrainConstants {
        private DriveTrainConstants() {}

        public static final int LEFT_LEAD_CAN_ID = 20;
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        public static final int RIGHT_LEAD_CAN_ID = 22;
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        public static final int CURRENT_LIMIT = 60;

        // 6-wheel drop-center drive: the center wheel sits 1/4" lower than
        // front/back, so each side only ever has 2 real ground contacts at once --
        // shorter effective wheelbase, less scrub while turning. Doesn't change the
        // kinematics below, which only cares about TRACK_WIDTH_METERS.
        public static final double DROP_CENTER_WHEEL_DROP_METERS = 0.25 * 0.0254;

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

        public static final double DRIVE_DISTANCE_KP = 1.5; // TODO: tune on the real robot -- starting point only
        public static final double DRIVE_DISTANCE_KI = 0.0;
        public static final double DRIVE_DISTANCE_KD = 0.1;
        public static final double DRIVE_DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double DRIVE_DISTANCE_MAX_OUTPUT = 0.6;
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

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

        public static final int CAM_MOTOR_ID = 31;

        public static final boolean CAM_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int CAM_CURRENT_LIMIT = 20;
        public static final double CAM_FIRE_SPEED = 0.6;

        public static final int LIMIT_SWITCH_DIO_PORT = 0;
        public static final int BEAM_BREAK_1_DIO_PORT = 1;
        public static final int BEAM_BREAK_2_DIO_PORT = 2;

        public static final boolean LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench (NC vs NO)
        public static final boolean BEAM_BREAK_1_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BEAM_BREAK_2_INVERTED = false; // TODO: verify polarity on bench

        public static final double FIRE_TIMEOUT_SECONDS = 2.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        public static final int LIFT_MOTOR_ID = 40;

        public static final boolean LIFT_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int LIFT_CURRENT_LIMIT = 30;

        public static final double RAISE_SPEED = 0.5;
        public static final double LOWER_SPEED = -0.7;

        public static final int TOP_LIMIT_SWITCH_DIO_PORT = 3;
        public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 4;
        public static final boolean TOP_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BOTTOM_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
    }

    public static final class GripperConstants {
        private GripperConstants() {}

        public static final int ROLLER_MOTOR_ID = 41;

        public static final boolean ROLLER_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int ROLLER_CURRENT_LIMIT = 20;

        public static final double INTAKE_SPEED = 1.0;
        public static final double EJECT_SPEED = -1.0;
    }

    public static final class Auto {
        private Auto() {}

        public static final double DRIVE_FORWARD_ONLY_FEET = 10.0;
        public static final double DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0;
        public static final double DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0;
        public static final double DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0;
    }
}
