package frc.robot.autonomous;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrain;

/** The two autonomous routines: wheel encoders and gyro only, no PathPlanner/vision. */
public final class AutoRoutines {
    private AutoRoutines() {}

    public static Command driveForwardOnly(DriveTrain drivetrain) {
        return new DriveDistanceCommand(drivetrain, DRIVE_FORWARD_ONLY_FEET);
    }

    public static Command driveTurnDrive(DriveTrain drivetrain) {
        return Commands.sequence(
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_FIRST_LEG_FEET),
            new TurnToAngleCommand(drivetrain, DRIVE_TURN_DRIVE_TURN_DEGREES),
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_SECOND_LEG_FEET)
        );
    }
}
