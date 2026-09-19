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
 * <p>Lives in {@code frc.robot.subsystems} (not {@code frc.robot}, where most of this
 * project's classes live) specifically so it can reach DriveTrain's package-private
 * {@code leftEncoder}/{@code rightEncoder} fields directly -- see those fields in
 * DriveTrain.java for why.
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

        DriveDistanceCommand command = new DriveDistanceCommand(drivetrain, 1.0); // 1 foot
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

        // Studica's AHRS exposes its simulated yaw through WPILib's SimDeviceSim
        // registry under "navX-Sensor[4]" rather than through a method on the AHRS
        // object itself.
        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");

        TurnToAngleCommand command = new TurnToAngleCommand(drivetrain, 90.0);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        // getHeadingDegrees() negates the raw navX yaw, so -90 raw yaw simulates
        // having reached +90 degrees heading.
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
