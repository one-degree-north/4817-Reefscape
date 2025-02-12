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

public class Elevator extends FSMSubsystem {
    // Constants
    private static final int ELEVATOR_MASTER_ID = 10;
    private static final int ELEVATOR_SLAVE_ID = 11;
    private static final int MAGNETIC_LIMIT_SWITCH_ID = 1;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;
    private static final double kV = 0.12;
    private static final double kA = 0.0;
    private static final double kG = 0.0;
    private static final double ELEVATOR_GEAR_RATIO = 1.0;
    private static final double mmAcceleration = 100.0;
    private static final double mmCruiseVelocity = 200.0;
    private static final double mmJerk = 1000.0;
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
                mmAcceleration,
                mmCruiseVelocity,
                mmJerk
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
    
        public boolean isElevatorAtGoal() {
            return Math.abs(m_elevatorMasterMotor.getPosition().getValueAsDouble() - 
                   ((ElevatorStates)getCurrentState()).getSetpointValue()) < ELEVATOR_ALLOWED_ERROR;
        }
    
        @Override
        protected void enterNewState() {
            ElevatorStates newState = (ElevatorStates)getCurrentState();
            setControl(m_elevatorMasterMotor, motionMagicVoltage.withPosition(newState.getSetpointValue()));
        }
    
        @Override
        protected void exitCurrentState() {
            // No specific exit actions needed
        }
    
        @Override
        protected void executeCurrentStateBehavior() {
            // Continuous behavior for the current state, if any
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
    
            SmartDashboard.putString("ElevatorState", getCurrentState().toString());
            SmartDashboard.putNumber("Elevator Position", m_elevatorMasterMotor.getPosition().getValueAsDouble());
        }
    
        public enum ElevatorStates {
            ELEVATOR_DOCKED(ELEVATOR_DOCKED_POS),
            ELEVATOR_L1(ELEVATOR_L1_POS),
            ELEVATOR_L2(ELEVATOR_L2_POS),
            ELEVATOR_L3(ELEVATOR_L3_POS),
            ELEVATOR_L4(ELEVATOR_L4_POS),
            ELEVATOR_HP(ELEVATOR_HP_POS);

        private final double setpointValue;

        ElevatorStates(double setpointValue) {
            this.setpointValue = setpointValue;
        }

        public double getSetpointValue() {
            return setpointValue;
        }
    }
}
