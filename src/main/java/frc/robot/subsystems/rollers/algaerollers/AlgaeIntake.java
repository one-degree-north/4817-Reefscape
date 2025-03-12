// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.algaerollers;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

@Logged
public class AlgaeIntake extends FSMSubsystem {
    // Constants
    private static final int ALGAE_INTAKE_MASTER_ID = 41; // Replace with actual ID
    private static final int ALGAE_INTAKE_SLAVE_ID = -1; // Replace with actual ID or -1 if no follower
    private static final double INTAKE_VOLTAGE = 1.5; // Voltage for intake
    private static final double OUTTAKE_VOLTAGE = -INTAKE_VOLTAGE; // Voltage for outtake
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
            "Falcon500",
            NeutralModeValue.Brake,
            InvertedValue.CounterClockwise_Positive,
            0.23861, null, null, 0.061375, 0.16521, 0.0028082, null, 1.2/1, null, null, null
        );

        if (hasFollower && m_algaeIntakeSlave != null) {
            TalonFXConfigurator.configureTalonFX(
                m_algaeIntakeSlave,
                "Falcon500",
                NeutralModeValue.Brake,
                InvertedValue.Clockwise_Positive,
                null, null, null, null, null, null, null, 1.2/1, null, null, null
            );

            // Set the follower motor to follow the primary motor
            m_algaeIntakeSlave.setControl(new Follower(m_algaeIntakeMaster.getDeviceID(), false));
        }
    }

    @Override
    protected void executeCurrentStateBehavior() {
        AlgaeIntakeStates newState = (AlgaeIntakeStates) getCurrentState();
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
            case IDLE:
                setTargetRPM(0);
            default:
                break;
        }
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
        AlgaeIntakeStates desiredState = (AlgaeIntakeStates) getDesiredState();
        
        if (desiredState == AlgaeIntakeStates.SHOOT) {
            return Math.abs(currentRPM - SHOOT_RPM) <= RPM_TOLERANCE;
        }
        
        return false; // Only check RPM for SHOOT state
    }

    public double getCurrentRPM() {
        return m_algaeIntakeMaster.getVelocity().getValueAsDouble() * 60;
    }

    private final SysIdRoutine algaeIntakeCharacterization = new SysIdRoutine(
        new SysIdRoutine.Config(null, Voltage.ofBaseUnits(3, Volt), null,
            (state)-> SignalLogger.writeString("AlgaeIntakeState", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
                m_algaeIntakeMaster.setControl(voltageOut.withOutput(volts));
            },
            null,
            this
        )
    );

    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return algaeIntakeCharacterization.quasistatic(direction);
    }

    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
        return algaeIntakeCharacterization.dynamic(direction);
    }

    public Command setGoalCommand(AlgaeIntakeStates goal) {
        return startEnd(()-> setGoal(goal), ()-> setGoal(AlgaeIntakeStates.IDLE));
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
        return AlgaeIntakeStates.values();
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method

        SmartDashboard.putString("Algae Intake State", getCurrentState().toString());
        SmartDashboard.putNumber("Algae Intake Current RPM", getCurrentRPM());
        SmartDashboard.putBoolean("Algae Intake At TargetRPM?", isAtTargetRPM());
    }

    public enum AlgaeIntakeStates {
        INTAKE,
        OUTTAKE,
        IDLE,
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
