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

public class DriveTrain extends SubsystemBase {

    private final Vision vision;

    private final SparkMax leftLead = new SparkMax(LEFT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax leftFollow = new SparkMax(LEFT_FOLLOW_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightLead = new SparkMax(RIGHT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightFollow = new SparkMax(RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);

    // Package-private (not private): DriveTrainTest.java, in this same package, pokes
    // these directly to fake driven distance in tests -- Java's `private` has no
    // bypassable loophole the way Python's underscore convention does.
    final RelativeEncoder leftEncoder = leftLead.getEncoder();
    final RelativeEncoder rightEncoder = rightLead.getEncoder();

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final DifferentialDrive driver = new DifferentialDrive(leftLead, rightLead);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    private final DifferentialDrivePoseEstimator poseEstimator;

    private final Field2d field = new Field2d();

    private int telemetryLoopCounter = 0;

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

    private void configureMotors() {
        double conversionFactor = WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO;

        SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        rightLeadConfig.inverted(true);
        rightLeadConfig.idleMode(IdleMode.kCoast);
        rightLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        rightLeadConfig.encoder.positionConversionFactor(conversionFactor);
        rightLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        rightLead.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        leftLeadConfig.inverted(false);
        leftLeadConfig.idleMode(IdleMode.kCoast);
        leftLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        leftLeadConfig.encoder.positionConversionFactor(conversionFactor);
        leftLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        leftLead.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void drive(double left, double right) {
        driver.tankDrive(
            MathUtil.applyDeadband(left, JOYSTICK_DEADBAND),
            MathUtil.applyDeadband(right, JOYSTICK_DEADBAND)
        );
    }

    public void stop() {
        driver.stopMotor();
    }

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

    // Public (not package-private) because ApproachTagCommandTest.java lives in a
    // different package (frc.robot.commands) and needs to fake driven distance too --
    // package-private access can't span two different packages.
    public void setEncoderPositionsForTest(double leftMeters, double rightMeters) {
        leftEncoder.setPosition(leftMeters);
        rightEncoder.setPosition(rightMeters);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getHeadingDegrees() {
        // navX reports clockwise-positive; this codebase's convention is CCW-positive.
        return -gyro.getAngle();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeadingDegrees()), 0.0, 0.0, pose);
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            Rotation2d.fromDegrees(getHeadingDegrees()), getLeftDistanceMeters(), getRightDistanceMeters()
        );

        Optional<VisionMeasurement> measurement = vision.getBestVisionMeasurementIfFresh();
        measurement.ifPresent(m ->
            poseEstimator.addVisionMeasurement(m.estimatedPose(), m.timestampSeconds(), m.standardDeviations())
        );

        field.setRobotPose(getPose());

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
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
