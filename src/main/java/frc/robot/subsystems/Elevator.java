package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final SparkMax liftMotor = new SparkMax(LIFT_MOTOR_ID, MotorType.kBrushed);

    private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_DIO_PORT);

    public Elevator() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.inverted(LIFT_MOTOR_INVERTED);
        liftConfig.idleMode(IdleMode.kBrake);
        liftConfig.smartCurrentLimit(LIFT_CURRENT_LIMIT);
        liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isAtTop() {
        boolean raw = topLimitSwitch.get();
        return TOP_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean isAtBottom() {
        boolean raw = bottomLimitSwitch.get();
        return BOTTOM_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public void setSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void stop() {
        liftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
    }
}
