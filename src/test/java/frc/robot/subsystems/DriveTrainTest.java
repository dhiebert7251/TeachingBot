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

// Lives in frc.robot.subsystems (not frc.robot) so it can reach DriveTrain's
// package-private leftEncoder/rightEncoder fields directly -- see those fields'
// comment in DriveTrain.java.
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
        enable();
        step(0.02);

        DriveDistanceCommand command = new DriveDistanceCommand(drivetrain, 1.0); // 1 foot
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled()); // nowhere near the target yet

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

        double drivenMeters = 1.0 * Constants.METERS_PER_FOOT;
        drivetrain.leftEncoder.setPosition(drivenMeters);
        drivetrain.rightEncoder.setPosition(drivenMeters);
        drivetrain.periodic();

        Pose2d poseAfterLegOne = drivetrain.getPose();
        assertEquals(drivenMeters, poseAfterLegOne.getX(), 0.001);

        // A second DriveDistanceCommand used to call drivetrain.resetEncoders() here,
        // which snapped the tracked pose back toward the origin -- regression test for
        // that fix (see DriveDistanceCommand.initialize()).
        DriveDistanceCommand secondLeg = new DriveDistanceCommand(drivetrain, 1.0);
        secondLeg.initialize();
        drivetrain.periodic();

        Pose2d poseAfterSecondLegStarts = drivetrain.getPose();
        assertEquals(drivenMeters, poseAfterSecondLegStarts.getX(), 0.001);
    }
}
