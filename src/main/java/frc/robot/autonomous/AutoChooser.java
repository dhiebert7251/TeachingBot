package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

/** Builds the SmartDashboard autonomous-routine chooser. */
public final class AutoChooser {
    private AutoChooser() {}

    private static final String DO_NOTHING_NAME = "Do Nothing";
    private static final String DRIVE_FORWARD_NAME = "Drive Forward 10 ft";
    private static final String DRIVE_TURN_DRIVE_NAME = "Drive 5ft, Turn Left 90, Drive 3ft";

    public static SendableChooser<Command> build(DriveTrain drivetrain) {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption(DO_NOTHING_NAME, Commands.none());
        chooser.addOption(DRIVE_FORWARD_NAME, AutoRoutines.driveForwardOnly(drivetrain));
        chooser.addOption(DRIVE_TURN_DRIVE_NAME, AutoRoutines.driveTurnDrive(drivetrain));
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }
}
