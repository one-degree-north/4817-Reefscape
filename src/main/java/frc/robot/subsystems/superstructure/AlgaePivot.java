// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class AlgaePivot extends FSMSubsystem {
    // Constants
    private static final int m_algaeIntakeID = 20; // Replace with actual ID
    private static final double m_dockedPosition = 0.0; // Replace with actual docked position
    private static final double m_intakingPosition = 10.0; // Replace with actual extended position
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;
    private static final double kV = 0.12;
    private static final double kA = 0.0;
    private static final double kG = 0.0;
    private static final double m_mechanismRatio = 1.0;
    private static final double m_mmAcceleration = 100.0;
    private static final double m_mmCruiseVelocity = 200.0;
    private static final double m_mmJerk = 1000.0;
    private static final double POSITION_TOLERANCE = 0.1;

    private TalonFX m_algaeIntakeMotor;
    private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    public static boolean isAlgaeIntaked = false;

    public AlgaePivot() {
        setName("AlgaeIntake");
        m_algaeIntakeMotor = new TalonFX(m_algaeIntakeID, "rio");
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfigurator.configureTalonFX(
            m_algaeIntakeMotor,
            "KrakenX60",
            NeutralModeValue.Brake,
            InvertedValue.Clockwise_Positive,
            kP, kI, kD, kS, kV, kA, kG,
            m_mechanismRatio,
            m_mmAcceleration,
            m_mmCruiseVelocity,
            m_mmJerk
        );
    }

    @Override
    protected void enterNewState() {
        AlgaeStates newState = (AlgaeStates)getCurrentState();
        setMotorPosition(newState.getSetpointValue());
    }

    @Override
    protected void exitCurrentState() {
        // No specific exit actions needed
    }

    @Override
    protected void executeCurrentStateBehavior() {
        // Continuous behavior for the current state, if any
    }

    private void setMotorPosition(double position) {
        if (m_algaeIntakeMotor.isAlive()) {
            m_algaeIntakeMotor.setControl(m_motionMagicVoltage.withPosition(position));
        }
    }

    public boolean isAtSetpoint() {
        AlgaeStates desiredState = (AlgaeStates) getDesiredState();
        double currentPosition = m_algaeIntakeMotor.getPosition().getValueAsDouble();
        double targetPosition = desiredState.getSetpointValue();
        
        return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
    }

    @Override
    public void stop() {
        m_algaeIntakeMotor.stopMotor();
    }

    @Override
    public boolean isInState(Enum<?> state) {
        return getCurrentState() == state;
    }

    @Override
    public Enum<?> getCurrentState() {
        return currentState;
    }

    @Override
    protected Enum<?>[] getStates() {
        return AlgaeStates.values();
    }

    @Override
    public Enum<?> getDesiredState() {
        return desiredState;
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method
        SmartDashboard.putString("AlgaeIntakeState", getCurrentState().toString());
        SmartDashboard.putNumber("AlgaeIntakePosition", m_algaeIntakeMotor.getPosition().getValueAsDouble());
    }

    public enum AlgaeStates {
        DOCKED(m_dockedPosition),
        INTAKING(m_intakingPosition);

        private final double setpointValue;

        AlgaeStates(double setpointValue) {
            this.setpointValue = setpointValue;
        }

        public double getSetpointValue() {
            return setpointValue;
        }
    }
}
