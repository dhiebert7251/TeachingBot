package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

/**
 * The cam has only one sensor (home). A full fire cycle is "leave home, then return
 * to home" -- tracked here with hasLeftHome, reset each time the command restarts.
 *
 * <p>No timeout of its own: a jam or broken switch would let it run forever, so the
 * safety timeout is applied via {@code .withTimeout()} where this is bound in
 * RobotContainer.java.
 */
public class FireCommand extends Command {

    private final Trigger trigger;
    private boolean hasLeftHome = false;

    public FireCommand(Trigger trigger) {
        this.trigger = trigger;
        addRequirements(trigger);
    }

    @Override
    public void initialize() {
        hasLeftHome = false;
    }

    @Override
    public void execute() {
        trigger.runCam();
    }

    @Override
    public boolean isFinished() {
        if (!hasLeftHome) {
            if (!trigger.isAtHome()) {
                hasLeftHome = true;
            }
            return false;
        }
        return trigger.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        trigger.stopCam();
    }
}
