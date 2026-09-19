package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveDistanceCommand extends Command {

    private final DriveTrain drivetrain;
    private final double targetDistanceMeters;
    private final PIDController pid;

    private double startDistanceMeters;

    public DriveDistanceCommand(DriveTrain drivetrain, double distanceFeet) {
        this.drivetrain = drivetrain;
        this.targetDistanceMeters = distanceFeet * METERS_PER_FOOT;
        addRequirements(drivetrain);

        pid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        pid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    @Override
    public void initialize() {
        // Records the current encoder reading as a baseline rather than calling
        // drivetrain.resetEncoders(): once DriveTrain's pose estimator started reading
        // these same encoders every loop, a mid-match reset silently corrupted the
        // pose (a real bug, since fixed by measuring distance relative to this
        // baseline instead of zeroing the hardware).
        startDistanceMeters = drivetrain.getAverageDistanceMeters();
        pid.reset();
        pid.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void execute() {
        double distanceThisLeg = drivetrain.getAverageDistanceMeters() - startDistanceMeters;
        double output = pid.calculate(distanceThisLeg);
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
