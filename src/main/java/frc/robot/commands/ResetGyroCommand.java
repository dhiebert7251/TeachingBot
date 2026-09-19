package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Resets the navX gyro's heading to 0 -- bound to the driver's Back button. Do this
 * before every autonomous run, with the robot pointed the way it should be for that
 * run's "0 degrees."
 */
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
