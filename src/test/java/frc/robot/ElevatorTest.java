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

        topSim.setValue(false);
        assertFalse(elevator.isAtTop());

        // Commands can only be scheduled while the robot is enabled.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        RaiseElevatorCommand command = new RaiseElevatorCommand(elevator);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        topSim.setValue(true);
        step(0.1);
        assertTrue(elevator.isAtTop());
    }

    @Test
    void lowerCommandStopsAtBottom() {
        DIOSim bottomSim = new DIOSim(Constants.ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT);

        bottomSim.setValue(false);
        assertFalse(elevator.isAtBottom());

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        LowerElevatorCommand command = new LowerElevatorCommand(elevator);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        bottomSim.setValue(true);
        step(0.1);
        assertTrue(elevator.isAtBottom());
    }
}
