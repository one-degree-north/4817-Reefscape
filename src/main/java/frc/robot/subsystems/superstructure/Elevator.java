// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class Elevator extends FSMSubsystem {
    // Constants
    private static final int ELEVATOR_MASTER_ID = 2;
    private static final int ELEVATOR_SLAVE_ID = 1;
    private static final int MAGNETIC_LIMIT_SWITCH_ID = 2;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;
    private static final double kV = 0.12;
    private static final double kA = 0.0;
    private static final double kG = 0.0;
    private static final double ELEVATOR_GEAR_RATIO = 39.2/1;
    private static final double MM_ACCELERATION = 0;
    private static final double MM_CRUISE_VELOCITY = 0;
    private static final double MM_JERK = 0;
    private static final double ELEVATOR_DOCKED_POS = 0.0;
    private static final double ELEVATOR_L1_POS = 10.0;
    private static final double ELEVATOR_L2_POS = 20.0;
    private static final double ELEVATOR_L3_POS = 30.0;
    private static final double ELEVATOR_L4_POS = 40.0;
    private static final double ELEVATOR_ALLOWED_ERROR = 0.5;
    private static final double ELEVATOR_HP_POS = 50.0;

    private TalonFX m_elevatorMasterMotor;
    private TalonFX m_elevatorSlaveMotor;
    private DigitalInput m_bottomLimitSwitch;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private VoltageOut voltageOut = new VoltageOut(0);
    
    private NeutralModeValue m_currentNeutralMode = NeutralModeValue.Brake;
    
    public Elevator() {
        setName("Elevator");
        m_elevatorMasterMotor = new TalonFX(ELEVATOR_MASTER_ID, "rio");
        m_elevatorSlaveMotor = new TalonFX(ELEVATOR_SLAVE_ID, "rio");
        m_bottomLimitSwitch = new DigitalInput(MAGNETIC_LIMIT_SWITCH_ID);
        motorConfigurations();
    }

    private void motorConfigurations() {
        TalonFXConfigurator.configureTalonFX(
            m_elevatorMasterMotor,
            "KrakenX60",
            m_currentNeutralMode,
            InvertedValue.Clockwise_Positive,
            kP, kI, kD, kS, kV, kA, kG,
            ELEVATOR_GEAR_RATIO,
            MM_ACCELERATION,
            MM_CRUISE_VELOCITY,
            MM_JERK
        );

        TalonFXConfigurator.configureTalonFX(
            m_elevatorSlaveMotor,
            "KrakenX60",
            m_currentNeutralMode,
            InvertedValue.Clockwise_Positive,
            null, null, null, null, null, null, null,
            null, 
            null, null, null
        );

        Follower followerConfig = new Follower(m_elevatorMasterMotor.getDeviceID(), true);
        m_elevatorSlaveMotor.setControl(followerConfig);
    }

    public boolean isElevatorDown() {
        return m_bottomLimitSwitch.get();
    }

    private void setControl(TalonFX motor, ControlRequest req) {
        if (motor.isAlive()) {
            motor.setControl(req);
        }
    }

    private void resetElevatorPosition() {
        m_elevatorMasterMotor.setPosition(0);
        m_elevatorSlaveMotor.setPosition(0);
    }

    private void toggleIdleMode() {
        m_currentNeutralMode = (m_currentNeutralMode == NeutralModeValue.Brake) 
            ? NeutralModeValue.Coast 
            : NeutralModeValue.Brake;
        motorConfigurations();
    }

    public void zeroAndToggleIdleMode() {
        resetElevatorPosition();
        toggleIdleMode();
    }

    private final SysIdRoutine elevatorCharacterization = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Second),
            Voltage.ofBaseUnits(3.5, Volt), 
            null,
            (state)-> SignalLogger.writeString("ElevatorState", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
                m_elevatorMasterMotor.setControl(voltageOut.withOutput(volts));
            },
            null,
            this
        )
    );

    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return elevatorCharacterization.quasistatic(direction);
    }

    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
        return elevatorCharacterization.dynamic(direction);
    }

    public boolean atGoal() {
        return Math.abs(m_elevatorMasterMotor.getPosition().getValueAsDouble() - 
            ((ElevatorStates)getCurrentState()).getSetpointValue()) < ELEVATOR_ALLOWED_ERROR;
    }

    @Override
    protected void executeCurrentStateBehavior() {
        ElevatorStates newState = (ElevatorStates)getCurrentState();
        setControl(m_elevatorMasterMotor, 
            motionMagicVoltage.withPosition(newState.getSetpointValue()));
    }

    @Override
    public void stop() {
        m_elevatorMasterMotor.stopMotor();
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
        return ElevatorStates.values();
    }

    @Override
    public void periodic() {
        update();

        SmartDashboard.putString("Elevator State", getCurrentState().toString());
        SmartDashboard.putNumber("Elevator Position", m_elevatorMasterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Is Elevator Down", isElevatorDown());
        SmartDashboard.putBoolean("Is Elevator AtGoal?", atGoal());
    }

    public enum ElevatorStates {
        DOCKED(ELEVATOR_DOCKED_POS),
        L1(ELEVATOR_L1_POS),
        L2(ELEVATOR_L2_POS),
        L3(ELEVATOR_L3_POS),
        L4(ELEVATOR_L4_POS),
        HUMAN_PLAYER(ELEVATOR_HP_POS);

    private final double setpointValue;

    ElevatorStates(double setpointValue) {
        this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
        return setpointValue;
    }
}
}
