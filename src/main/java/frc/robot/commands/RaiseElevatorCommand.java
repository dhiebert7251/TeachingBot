package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.RAISE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

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
