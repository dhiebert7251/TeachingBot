package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

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
