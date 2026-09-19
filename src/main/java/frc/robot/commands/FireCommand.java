package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

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
