package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoChooser;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trigger;

/**
 * Wires the five subsystems together, sets teleop default commands and button
 * bindings, and builds the autonomous chooser. See README.md for the full
 * controller-binding table.
 */
public class RobotContainer {

    // Public, not private: Robot.java and tests need to reach these subsystems
    // directly (e.g. to poke simulated hardware).
    public final DriveTrain drivetrain = new DriveTrain();
    public final Shooter shooter = new Shooter();
    public final Trigger trigger = new Trigger();
    public final Elevator elevator = new Elevator();
    public final Gripper gripper = new Gripper();

    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();

        autoChooser = AutoChooser.build(drivetrain);
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(drivetrain, driverController));
    }

    /**
     * Driver (port 0): drive only. Operator (port 1): everything else. See README.md
     * for the full button map.
     */
    private void configureBindings() {
        driverController.back().onTrue(new ResetGyroCommand(drivetrain));

        operatorController.a().toggleOnTrue(new SpinUpShooterCommand(shooter));

        // Safety timeout applied here, not inside FireCommand -- see that class.
        operatorController.b().onTrue(new FireCommand(trigger).withTimeout(FIRE_TIMEOUT_SECONDS));

        operatorController.x().whileTrue(new IntakeCommand(gripper));
        operatorController.y().whileTrue(new EjectCommand(gripper));
        operatorController.rightBumper().whileTrue(new RaiseElevatorCommand(elevator));
        operatorController.leftBumper().whileTrue(new LowerElevatorCommand(elevator));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
