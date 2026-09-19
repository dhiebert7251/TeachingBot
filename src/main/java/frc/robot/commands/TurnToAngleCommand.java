package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleCommand extends Command {

    private final DriveTrain drivetrain;
    private final double targetDegrees;
    private final PIDController pid;

    public TurnToAngleCommand(DriveTrain drivetrain, double targetDegrees) {
        this.drivetrain = drivetrain;
        this.targetDegrees = targetDegrees;
        addRequirements(drivetrain);

        pid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(TURN_TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(targetDegrees);
    }

    @Override
    public void execute() {
        double output = pid.calculate(drivetrain.getHeadingDegrees());
        drivetrain.drive(-output, output);
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
