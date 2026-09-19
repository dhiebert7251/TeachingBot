package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.SPEED_SCALE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command {

    private final DriveTrain drivetrain;
    private final CommandXboxController driverController;

    public TeleopDriveCommand(DriveTrain drivetrain, CommandXboxController driverController) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double leftY = -driverController.getLeftY();
        double rightY = -driverController.getRightY();
        drivetrain.drive(SPEED_SCALE * leftY, SPEED_SCALE * rightY);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
