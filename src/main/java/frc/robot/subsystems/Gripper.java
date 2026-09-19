package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

    public Gripper() {
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(ROLLER_MOTOR_INVERTED);
        rollerConfig.idleMode(IdleMode.kBrake);
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0.0);
    }
}
