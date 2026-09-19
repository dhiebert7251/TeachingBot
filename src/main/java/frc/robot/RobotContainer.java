package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoChooser;
import frc.robot.commands.ApproachTagCommand;
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
import frc.robot.subsystems.Vision;

public class RobotContainer {

    public final Vision vision = new Vision();
    public final DriveTrain drivetrain = new DriveTrain(vision);
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

    private void configureBindings() {
        driverController.back().onTrue(new ResetGyroCommand(drivetrain));

        driverController.a().onTrue(
            new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, APPROACH_STANDOFF_FEET)
        );
        driverController.x().onTrue(
            new ApproachTagCommand(
                drivetrain,
                vision,
                EXAMPLE_TAG_ID,
                APPROACH_AND_TURN_STANDOFF_FEET,
                APPROACH_AND_TURN_OFFSET_DEGREES
            )
        );

        operatorController.a().toggleOnTrue(new SpinUpShooterCommand(shooter));

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
