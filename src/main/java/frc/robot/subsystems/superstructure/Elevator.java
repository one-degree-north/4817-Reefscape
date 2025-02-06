// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

@Logged
public class Elevator extends FSMSubsystem {
    // Constants
    private static final int m_elevatorMasterID = 10;
    private static final int m_elevatorSlaveID = 11;
    private static final int m_magneticLimitSwitchID = 1;
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
    private static final double m_dockedPos = 0.0;
    private static final double m_l1Pos = 10.0;
    private static final double m_l2Pos = 20.0;
    private static final double m_l3Pos = 30.0;
    private static final double m_l4Pos = 40.0;
    private static final double m_allowedError = 0.5;

    private TalonFX m_elevatorMasterMotor;
    private TalonFX m_elevatorSlaveMotor;
    private DigitalInput m_bottomLimitSwitch;

    private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    
    private NeutralModeValue m_currentNeutralMode = NeutralModeValue.Brake;

    public Elevator() {
        setName("Elevator");
        m_elevatorMasterMotor = new TalonFX(m_elevatorMasterID, "rio");
        m_elevatorSlaveMotor = new TalonFX(m_elevatorSlaveID, "rio");
        m_bottomLimitSwitch = new DigitalInput(m_magneticLimitSwitchID);
        motorConfigurations();
    }

    private void motorConfigurations() {
        TalonFXConfigurator.configureTalonFX(
            m_elevatorMasterMotor,
            "KrakenX60",
            m_currentNeutralMode,
            InvertedValue.Clockwise_Positive,
            kP, kI, kD, kS, kV, kA, kG,
            m_mechanismRatio,
            m_mmAcceleration,
            m_mmCruiseVelocity,
            m_mmJerk
        );

        TalonFXConfigurator.configureTalonFX(
            m_elevatorSlaveMotor,
            "KrakenX60",
            m_currentNeutralMode,
            InvertedValue.Clockwise_Positive,
            null, null, null, null, null, null, null, null, null, null, null
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

    public void resetElevatorPosition() {
        m_elevatorMasterMotor.setPosition(0);
        m_elevatorSlaveMotor.setPosition(0);
    }

    public void toggleIdleMode() {
        m_currentNeutralMode = (m_currentNeutralMode == NeutralModeValue.Brake) 
            ? NeutralModeValue.Coast 
            : NeutralModeValue.Brake;

        motorConfigurations();

        SmartDashboard.putString("Elevator Idle Mode", m_currentNeutralMode.toString());
    }

    private final SysIdRoutine elevatorCharacterization = new SysIdRoutine(
        new SysIdRoutine.Config(null, Voltage.ofBaseUnits(3, Volt), null),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
                m_elevatorMasterMotor.setVoltage(volts.in(Volt));
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

    public boolean isElevatorAtSetpoint() {
        return Math.abs(m_elevatorMasterMotor.getPosition().getValueAsDouble() - 
               ((ElevatorStates)getCurrentState()).getSetpointValue()) < m_allowedError;
    }

    @Override
    protected void enterNewState() {
        ElevatorStates newState = (ElevatorStates)getCurrentState();
        setControl(m_elevatorMasterMotor, m_motionMagicVoltage.withPosition(newState.getSetpointValue()));
    }

    @Override
    protected void exitCurrentState() {
        // No specific exit actions needed
    }

    @Override
    protected void executeCurrentStateBehavior() {
        // Continuous behavior for the current state, if any
    }

    public enum ElevatorStates {
        ELEVATOR_DOCKED(m_dockedPos),
        ELEVATOR_L1(m_l1Pos),
        ELEVATOR_L2(m_l2Pos),
        ELEVATOR_L3(m_l3Pos),
        ELEVATOR_L4(m_l4Pos);

        private final double setpointValue;

        ElevatorStates(double setpointValue) {
            this.setpointValue = setpointValue;
        }

        public double getSetpointValue() {
            return setpointValue;
        }
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method

        SmartDashboard.putString("ElevatorState", getCurrentState().toString());
        SmartDashboard.putNumber("Elevator Position", m_elevatorMasterMotor.getPosition().getValueAsDouble());
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
    protected Enum<?>[] getStates() {
        return ElevatorStates.values();
    }
}
