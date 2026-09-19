package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.TARGET_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * No execute(): Phoenix 6 velocity control is closed-loop on the TalonFX itself, so
 * this only needs to set the target once and stop once.
 */
public class SpinUpShooterCommand extends Command {

    private final Shooter shooter;

    public SpinUpShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(TARGET_RPM);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
