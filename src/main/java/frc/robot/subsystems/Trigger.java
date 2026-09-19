package frc.robot.subsystems;

import static frc.robot.Constants.TriggerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trigger extends SubsystemBase {

    private final SparkMax camMotor = new SparkMax(CAM_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput beamBreak1 = new DigitalInput(BEAM_BREAK_1_DIO_PORT);
    private final DigitalInput beamBreak2 = new DigitalInput(BEAM_BREAK_2_DIO_PORT);

    public Trigger() {
        SparkMaxConfig camConfig = new SparkMaxConfig();
        camConfig.inverted(CAM_MOTOR_INVERTED);
        camConfig.idleMode(IdleMode.kBrake);
        camConfig.smartCurrentLimit(CAM_CURRENT_LIMIT);
        camMotor.configure(camConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isAtHome() {
        boolean raw = limitSwitch.get();
        return LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage1() {
        boolean raw = beamBreak1.get();
        return BEAM_BREAK_1_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage2() {
        boolean raw = beamBreak2.get();
        return BEAM_BREAK_2_INVERTED ? !raw : raw;
    }

    public void runCam() {
        camMotor.set(CAM_FIRE_SPEED);
    }

    public void stopCam() {
        camMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Trigger/AtHome", isAtHome());
        SmartDashboard.putBoolean("Trigger/BallStage1", hasBallAtStage1());
        SmartDashboard.putBoolean("Trigger/BallStage2", hasBallAtStage2());
    }
}
