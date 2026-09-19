package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.SPEED_SCALE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

/**
 * The default command: tank drive read straight from the driver controller's two
 * joystick Y-axes.
 */
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
        // Xbox joysticks report "pushed forward" as a negative Y value; the leading
        // minus signs flip that back.
        double leftY = -driverController.getLeftY();
        double rightY = -driverController.getRightY();
        drivetrain.drive(SPEED_SCALE * leftY, SPEED_SCALE * rightY);
    }

    @Override
    public boolean isFinished() {
        // Runs forever, until another command that also needs DriveTrain interrupts it.
        return false;
    }
}
