// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.algaerollers;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

@Logged
public class AlgaeIntake extends FSMSubsystem {
    // Constants
    private static final int ALGAE_INTAKE_MASTER_ID = 40; // Replace with actual ID
    private static final int ALGAE_INTAKE_SLAVE_ID = -1; // Replace with actual ID or -1 if no follower
    private static final double INTAKE_VOLTAGE = 6.0; // Voltage for intake
    private static final double OUTTAKE_VOLTAGE = -6.0; // Voltage for outtake
    private static final double SHOOT_RPM = 5000.0; // RPM for shooting
    private static final double RPM_TOLERANCE = 50.0; // Tolerance for reaching desired RPM

    private TalonFX m_algaeIntakeMaster;
    private TalonFX m_algaeIntakeSlave;
    private VoltageOut voltageOut = new VoltageOut(0);
    private VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

    public AlgaeIntake(boolean hasFollower) {
        setName("Intake");
        m_algaeIntakeMaster = new TalonFX(ALGAE_INTAKE_MASTER_ID, "rio");
        if (hasFollower) {
            m_algaeIntakeSlave = new TalonFX(ALGAE_INTAKE_SLAVE_ID, "rio");
        }
        configureMotors(hasFollower);
    }

    private void configureMotors(boolean hasFollower) {
        // Configure the primary motor
        TalonFXConfigurator.configureTalonFX(
            m_algaeIntakeMaster,
            "KrakenX60",
            NeutralModeValue.Brake,
            InvertedValue.Clockwise_Positive,
            null, null, null, null, null, null, null, null, null, null, null
        );

        if (hasFollower && m_algaeIntakeSlave != null) {
            TalonFXConfigurator.configureTalonFX(
                m_algaeIntakeSlave,
                "KrakenX60",
                NeutralModeValue.Brake,
                InvertedValue.Clockwise_Positive,
                null, null, null, null, null, null, null, null, null, null, null
            );

            // Set the follower motor to follow the primary motor
            m_algaeIntakeSlave.setControl(new Follower(m_algaeIntakeMaster.getDeviceID(), false));
        }
    }

    @Override
    protected void enterNewState() {
        IntakeStates newState = (IntakeStates) getCurrentState();
        switch (newState) {
            case INTAKE:
                setVoltage(INTAKE_VOLTAGE);
                break;
            case OUTTAKE:
                setVoltage(OUTTAKE_VOLTAGE);
                break;
            case SHOOT:
                setTargetRPM(SHOOT_RPM);
                break;
        }
    }

    @Override
    protected void exitCurrentState() {
        // No specific exit actions needed
    }

    @Override
    protected void executeCurrentStateBehavior() {
        // Continuous behavior for the current state (if any)
    }

    private void setVoltage(double voltage) {
        if (m_algaeIntakeMaster.isAlive()) {
            m_algaeIntakeMaster.setControl(voltageOut.withOutput(voltage));
        }
    }

    private void setTargetRPM(double targetRPM) {
        if (m_algaeIntakeMaster.isAlive()) {
            double velocitySetpointRPS = targetRPM / 60;
            m_algaeIntakeMaster.setControl(velocityControl.withVelocity(velocitySetpointRPS));
        }
    }

    public boolean isAtTargetRPM() {
        double currentRPM = getCurrentRPM();
        IntakeStates desiredState = (IntakeStates) getDesiredState();
        
        if (desiredState == IntakeStates.SHOOT) {
            return Math.abs(currentRPM - SHOOT_RPM) <= RPM_TOLERANCE;
        }
        
        return false; // Only check RPM for SHOOT state
    }

    public double getCurrentRPM() {
        return m_algaeIntakeMaster.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void stop() {
        m_algaeIntakeMaster.stopMotor();
        if (m_algaeIntakeSlave != null) {
            m_algaeIntakeSlave.stopMotor();
        }
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
    public Enum<?> getDesiredState() {
        return desiredState;
    }

    @Override
    protected Enum<?>[] getStates() {
        return IntakeStates.values();
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method

        SmartDashboard.putString("IntakeState", getCurrentState().toString());
        SmartDashboard.putNumber("IntakeCurrentRPM", getCurrentRPM());
        SmartDashboard.putBoolean("IntakeAtTargetRPM", isAtTargetRPM());
    }

    public enum IntakeStates {
        INTAKE,
        OUTTAKE,
        SHOOT;

        public boolean usesVoltageControl() {
            return this == INTAKE || this == OUTTAKE;
        }
        
        public boolean usesVelocityControl() {
            return this == SHOOT;
        }
        
        public double getTargetValue() {
            switch (this) {
                case INTAKE:
                    return INTAKE_VOLTAGE;
                case OUTTAKE:
                    return OUTTAKE_VOLTAGE;
                case SHOOT:
                    return SHOOT_RPM;
                default:
                    return 0;
            }
        }
    }
}
