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
 * Lives in frc.robot.subsystems (not frc.robot) specifically to reach DriveTrain's
 * package-private leftEncoder/rightEncoder fields directly -- see DriveTrain.java.
 * No physics simulation is wired in, so the odometry tests below call
 * drivetrain.periodic() directly after poking sensor values.
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
        enable();
        step(0.02);

        DriveDistanceCommand command = new DriveDistanceCommand(drivetrain, 1.0);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

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

        // Studica's AHRS exposes simulated yaw via WPILib's SimDeviceSim registry
        // under "navX-Sensor[4]", not through a method on the AHRS object itself.
        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");

        TurnToAngleCommand command = new TurnToAngleCommand(drivetrain, 90.0);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        // getHeadingDegrees() negates raw navX yaw, so -90 raw yaw simulates +90 heading.
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
        // Heading left at 0, so odometry should report movement straight along X.
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
}
