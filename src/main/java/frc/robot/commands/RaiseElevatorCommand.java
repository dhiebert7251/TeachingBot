package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.RAISE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Bound with whileTrue; execute() also checks the limit switch directly so the
 * mechanism stops the instant it's reached, without waiting on the operator to
 * release the button.
 */
public class RaiseElevatorCommand extends Command {

    private final Elevator elevator;

    public RaiseElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtTop()) {
            elevator.stop();
        } else {
            elevator.setSpeed(RAISE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
