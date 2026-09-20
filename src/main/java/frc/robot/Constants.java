package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

        public static final int CURRENT_LIMIT = 60; // amps

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

        public static final double TURN_KP = 0.04;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.005;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;

        public static final double DRIVE_DISTANCE_KP = 1.5; // TODO: tune on the real robot -- starting point only
        public static final double DRIVE_DISTANCE_KI = 0.0;
        public static final double DRIVE_DISTANCE_KD = 0.1;
        public static final double DRIVE_DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double DRIVE_DISTANCE_MAX_OUTPUT = 0.6; // clamp: never command more than 60% power
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

        public static final int FLYWHEEL_MOTOR_ID = 30;

        public static final boolean FLYWHEEL_INVERTED = false; // TODO: verify on bench

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

        public static final int CAM_MOTOR_ID = 31;

        public static final boolean CAM_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int CAM_CURRENT_LIMIT = 20; // amps, small NEO
        public static final double CAM_FIRE_SPEED = 0.6; // [-1, 1] duty cycle while firing

        public static final int LIMIT_SWITCH_DIO_PORT = 0;
        public static final int BEAM_BREAK_1_DIO_PORT = 1; // e.g. "ball loaded, waiting to fire"
        public static final int BEAM_BREAK_2_DIO_PORT = 2; // e.g. "ball at the shooter, ready to fire"

        public static final boolean LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench (NC vs NO)
        public static final boolean BEAM_BREAK_1_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BEAM_BREAK_2_INVERTED = false; // TODO: verify polarity on bench

        public static final double FIRE_TIMEOUT_SECONDS = 2.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        public static final int LIFT_MOTOR_ID = 40;

        public static final boolean LIFT_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int LIFT_CURRENT_LIMIT = 30; // amps -- Redline motors are small, keep this conservative

        public static final double RAISE_SPEED = 0.5; // [-1, 1] duty cycle while raising (spring-assisted)
        public static final double LOWER_SPEED = -0.7; // [-1, 1] duty cycle while lowering (against spring tension)

        public static final int TOP_LIMIT_SWITCH_DIO_PORT = 3;
        public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 4;
        public static final boolean TOP_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BOTTOM_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
    }

    public static final class GripperConstants {
        private GripperConstants() {}

        public static final int ROLLER_MOTOR_ID = 41;

        public static final boolean ROLLER_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int ROLLER_CURRENT_LIMIT = 20; // amps, small NEO 550

        public static final double INTAKE_SPEED = 1.0;
        public static final double EJECT_SPEED = -1.0;
    }

    public static final class VisionConstants {
        private VisionConstants() {}

        public static final String CAMERA_NAME = "Front_Camera"; // TODO: must match PhotonVision UI camera name

        // Sign convention taken from wpimath's documented Rotation3d pitch behavior, not independently verified by running it -- see README.
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d((16.0 - 3.0) * 0.0254, 0.0, 1.0 * 0.3048),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)
        ); // TODO: camera mounting measurements need bench verification

        public static final double MAX_TAG_DISTANCE_METERS = 4.0; // TODO: tune based on camera performance
        public static final double MAX_AMBIGUITY = 0.3; // TODO: tune based on field testing
        public static final int MIN_TAGS_FOR_MULTI_TAG = 2;
        public static final double MAX_VISION_AGE_SECONDS = 0.5;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;

        public static final Matrix<N3, N1> SINGLE_TAG_CLOSE_STDDEVS =
            VecBuilder.fill(0.5, 0.5, Math.toRadians(10)); // TODO: tune based on testing
        public static final Matrix<N3, N1> SINGLE_TAG_FAR_STDDEVS =
            VecBuilder.fill(1.0, 1.0, Math.toRadians(20)); // TODO: tune based on testing
        public static final Matrix<N3, N1> MULTI_TAG_STDDEVS =
            VecBuilder.fill(0.2, 0.2, Math.toRadians(5)); // TODO: tune based on testing

        public static final int EXAMPLE_TAG_ID = 15;
        public static final double APPROACH_STANDOFF_FEET = 3.0; // stop this far away, facing the tag directly
        public static final double APPROACH_AND_TURN_STANDOFF_FEET = 5.0; // stop this far away, then rotate
        public static final double APPROACH_AND_TURN_OFFSET_DEGREES = 45.0; // positive = right (clockwise) of facing the tag
    }

    public static final class Auto {
        private Auto() {}

        public static final double DRIVE_FORWARD_ONLY_FEET = 10.0;
        public static final double DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0;
        public static final double DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0; // positive = left (CCW)
        public static final double DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0;
    }
}
