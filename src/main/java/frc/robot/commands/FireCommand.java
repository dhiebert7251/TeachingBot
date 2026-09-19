package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

/**
 * A full fire cycle is: leave home, then return to home -- reading the single home
 * limit switch just once would immediately (and wrongly) report "done," since the
 * cam starts every cycle already there. {@code isFinished()} tracks that edge with
 * {@code hasLeftHome}, reset every time this command restarts in {@code initialize()}.
 *
 * <p>This class sets no timeout of its own: a jammed cam or broken switch wire would
 * otherwise run it forever. The safety timeout is applied as a {@code .withTimeout()}
 * decorator at the one place this command is bound to a button, in
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
            // Still waiting for the cam to leave home for the first time -- once it
            // does, start watching for it to come back.
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
