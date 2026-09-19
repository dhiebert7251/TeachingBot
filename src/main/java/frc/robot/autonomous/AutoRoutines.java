package frc.robot.autonomous;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * The two autonomous routines for the teaching-bot proof of concept, built entirely
 * from commands/DriveDistanceCommand.java and commands/TurnToAngleCommand.java -- no
 * PathPlanner, no vision, just wheel encoders and a gyro.
 */
public final class AutoRoutines {
    private AutoRoutines() {}

    /** Drives straight forward {@code Auto.DRIVE_FORWARD_ONLY_FEET} feet, then stops. */
    public static Command driveForwardOnly(DriveTrain drivetrain) {
        return new DriveDistanceCommand(drivetrain, DRIVE_FORWARD_ONLY_FEET);
    }

    /**
     * Drives forward, turns, drives forward again:
     * {@code Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET} feet -&gt; turn
     * {@code Auto.DRIVE_TURN_DRIVE_TURN_DEGREES} degrees (positive = left) -&gt;
     * {@code Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET} feet.
     */
    public static Command driveTurnDrive(DriveTrain drivetrain) {
        return Commands.sequence(
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_FIRST_LEG_FEET),
            new TurnToAngleCommand(drivetrain, DRIVE_TURN_DRIVE_TURN_DEGREES),
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_SECOND_LEG_FEET)
        );
    }
}
