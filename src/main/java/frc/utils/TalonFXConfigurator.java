package frc.utils;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXConfigurator {

    /**
     * Configures a TalonFX motor with the provided parameters
     * 
     * @param talonFX The TalonFX motor instance to configure
     * @param motorType The type of motor ("KrakenX60" or "Falcon500")
     * @param neutralMode The neutral mode setting (e.g., Brake or Coast).
     * 
     * Set to null to skip configuration for the following parameters:
     *
     * @param inverted The inversion setting of the motor
     * @param kP Proportional gain for PID control
     * @param kI Integral gain for PID control
     * @param kD Derivative gain for PID control
     * @param kS Static gain for feedforward control
     * @param kV Velocity gain for feedforward control
     * @param kA Acceleration gain for feedforward control
     * @param kG Gravity gain for feedforward control
     * @param gravityTypeValue Gravity type for feedforward control
     * @param sensorToMechanismRatio The ratio between the sensor and the mechanism
     * @param motionMagicAcceleration Motion Magic acceleration value
     * @param motionMagicCruiseVelocity Motion Magic cruise velocity value
     * @param motionMagicJerk Motion Magic jerk value
     */
    public static void configureTalonFX(
            TalonFX talonFX,
            String motorType,
            NeutralModeValue neutralMode,
            InvertedValue inverted,
            GravityTypeValue gravityTypeValue,
            Double kP,
            Double kI,
            Double kD,
            Double kS,
            Double kV,
            Double kA,
            Double kG,
            Double sensorToMechanismRatio,
            Double motionMagicAcceleration,
            Double motionMagicCruiseVelocity,
            Double motionMagicJerk) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure current limits based on motor type
        if ("KrakenX60".equalsIgnoreCase(motorType)) {
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true);
            config.withCurrentLimits(currentLimits);
        } else if ("Falcon500".equalsIgnoreCase(motorType)) {
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(30)
                    .withStatorCurrentLimitEnable(true);
            config.withCurrentLimits(currentLimits);
        }

        // Configure motor output if parameters are provided
        if (neutralMode != null || inverted != null) {
            MotorOutputConfigs motorOutput = new MotorOutputConfigs();
            if (neutralMode != null) {
                motorOutput.withNeutralMode(neutralMode);
            }
            if (inverted != null) {
                motorOutput.withInverted(inverted);
            }
            config.withMotorOutput(motorOutput);
        }

        // Configure PID gains if parameters are provided
        if (kP != null || kI != null || kD != null || kS != null || kV != null || kA != null || kG != null || gravityTypeValue != null) {
            Slot0Configs slot0 = new Slot0Configs();
            if (kP != null) slot0.withKP(kP);
            if (kI != null) slot0.withKI(kI);
            if (kD != null) slot0.withKD(kD);
            if (kS != null) slot0.withKS(kS);
            if (kV != null) slot0.withKV(kV);
            if (kA != null) slot0.withKA(kA);
            if (kG != null) slot0.withKG(kG);
            if (gravityTypeValue != null) slot0.withGravityType(gravityTypeValue);
            config.withSlot0(slot0);
        }

        // Configure feedback if parameter is provided
        if (sensorToMechanismRatio != null) {
            FeedbackConfigs feedback = new FeedbackConfigs()
                    .withSensorToMechanismRatio(sensorToMechanismRatio);
            config.withFeedback(feedback);
        }

        // Configure Motion Magic if parameters are provided
        if (motionMagicAcceleration != null || motionMagicCruiseVelocity != null || motionMagicJerk != null) {
            MotionMagicConfigs motionMagic = new MotionMagicConfigs();
            if (motionMagicAcceleration != null) motionMagic.withMotionMagicAcceleration(motionMagicAcceleration);
            if (motionMagicCruiseVelocity != null) motionMagic.withMotionMagicCruiseVelocity(motionMagicCruiseVelocity);
            if (motionMagicJerk != null) motionMagic.withMotionMagicJerk(motionMagicJerk);
            config.withMotionMagic(motionMagic);
        }

        // Apply the configuration to the TalonFX
        talonFX.getConfigurator().apply(config);
    }
}
