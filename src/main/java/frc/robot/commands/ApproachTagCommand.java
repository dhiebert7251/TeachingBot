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

// TODO: no .withTimeout() on either binding of this command -- if the PID chain never
// converges, the driver can't preempt it with the default teleop command.
public class ApproachTagCommand extends Command {

    // Package-private: ApproachTagCommandTest.java (same package) reads these directly
    // to check state-machine progress -- Java's `private` has no test-bypass loophole.
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

    private final double faceOffsetDegrees;

    // Package-private for the same test-access reason as Phase above.
    final PIDController turnPid;
    final PIDController drivePid;

    Phase phase = Phase.DONE;
    Translation2d targetPoint = new Translation2d();
    private double finalHeadingDegrees = 0.0;
    private double driveStartDistanceMeters = 0.0;

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

    public ApproachTagCommand(DriveTrain drivetrain, Vision vision, int tagId, double standoffFeet) {
        this(drivetrain, vision, tagId, standoffFeet, 0.0);
    }

    @Override
    public void initialize() {
        Optional<Pose3d> tagPose = vision.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            phase = Phase.FAILED;
            return;
        }

        Pose2d tagPose2d = tagPose.get().toPose2d();
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
        double distanceMeters = drivetrain.getPose().getTranslation().getDistance(targetPoint);
        // Baseline measured here, not a resetEncoders() call -- see DriveDistanceCommand
        // for why resetting the encoders mid-match corrupts the pose estimator.
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
