package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
