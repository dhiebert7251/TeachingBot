package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Shooter subsystem -- single flywheel (Kraken/TalonFX), fixed target RPM. */
public class Shooter extends SubsystemBase {

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private double targetRpm = 0.0;

    private final TalonFX flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);

    public Shooter() {
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(FLYWHEEL_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(SHOOTER_KP)
                    .withKI(SHOOTER_KI)
                    .withKD(SHOOTER_KD)
                    .withKV(SHOOTER_KV));

        StatusCode configError = flywheelMotor.getConfigurator().apply(flywheelConfig);
        if (!configError.isOK()) {
            DriverStation.reportWarning("Shooter flywheel motor config failed: " + configError, false);
        }
    }

    /**
     * Closed-loop velocity control runs on the TalonFX itself, so this only needs to
     * be called once when the target changes, not every loop.
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        flywheelMotor.setControl(velocityRequest.withVelocity(rpm / FLYWHEEL_GEAR_RATIO / 60.0));
    }

    public void stop() {
        targetRpm = 0.0;
        flywheelMotor.setControl(neutralRequest);
    }

    public double getCurrentRpm() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0 * FLYWHEEL_GEAR_RATIO;
    }

    public boolean isAtTargetSpeed() {
        return targetRpm > 0 && Math.abs(getCurrentRpm() - targetRpm) <= RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/CurrentRPM", getCurrentRpm());
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtTargetSpeed());
    }
}
