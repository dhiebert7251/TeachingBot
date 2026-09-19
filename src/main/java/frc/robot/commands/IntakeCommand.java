package frc.robot.commands;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class IntakeCommand extends Command {

    private final Gripper gripper;

    public IntakeCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpeed(INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
