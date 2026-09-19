package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TriggerTest {

    private RobotContainer robotContainer;
    private Trigger trigger;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        trigger = robotContainer.trigger;
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
    void fireCommandFinishesWhenCamReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        limitSwitchSim.setValue(true);
        assertTrue(trigger.isAtHome());

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        FireCommand command = new FireCommand(trigger);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        limitSwitchSim.setValue(false);
        step(0.1);
        assertTrue(command.isScheduled());

        limitSwitchSim.setValue(true);
        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void fireCommandTimesOutIfNeverReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        limitSwitchSim.setValue(false); // never at home -- simulates a jam
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        // The wrapper .withTimeout() returns, not the inner FireCommand, is what's
        // actually scheduled -- checking the inner command's isScheduled() here would
        // read false from the first loop and pass without testing the timeout at all.
        Command timedCommand = new FireCommand(trigger).withTimeout(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS);
        timedCommand.schedule();
        step(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS + 0.5);
        assertFalse(timedCommand.isScheduled());
    }
}
