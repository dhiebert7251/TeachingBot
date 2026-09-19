package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.TARGET_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Only one command for Shooter: spin the flywheel to a fixed target speed and hold
 * it until interrupted. No {@code execute()} -- Phoenix 6's velocity control is
 * closed-loop on the TalonFX itself, so there's nothing to redo every loop.
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
