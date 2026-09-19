package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Drives straight to a target distance, given in feet, using a PID loop on the
 * average of the two drive encoders. PID (rather than a fixed speed until the target
 * is reached) tapers the commanded speed as the target nears instead of overshooting
 * at full power.
 */
public class DriveDistanceCommand extends Command {

    private final DriveTrain drivetrain;
    private final double targetDistanceMeters;
    private final PIDController pid;

    public DriveDistanceCommand(DriveTrain drivetrain, double distanceFeet) {
        this.drivetrain = drivetrain;
        this.targetDistanceMeters = distanceFeet * METERS_PER_FOOT;
        addRequirements(drivetrain);

        pid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        pid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        pid.reset();
        pid.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void execute() {
        // Clamped to +/-DRIVE_DISTANCE_MAX_OUTPUT as a safety margin independent of
        // how conservatively KP is tuned.
        double output = pid.calculate(drivetrain.getAverageDistanceMeters());
        output = Math.max(-DRIVE_DISTANCE_MAX_OUTPUT, Math.min(DRIVE_DISTANCE_MAX_OUTPUT, output));
        drivetrain.drive(output, output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
