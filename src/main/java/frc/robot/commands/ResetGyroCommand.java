package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/** Resets the navX heading to 0. Do this before every autonomous run. */
public class ResetGyroCommand extends Command {

    private final DriveTrain drivetrain;

    public ResetGyroCommand(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
