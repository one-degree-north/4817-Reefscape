// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class AlgaePivot extends FSMSubsystem {
// Constants
private static final int ALGAE_PIVOT_MASTER_ID = 42; // Replace with actual ID
private static final int ALGAE_PIVOT_SLAVE_ID = 6;

private static final double DOCKED_POSITION = 0.000; // Replace with actual docked position
private static final double INTAKING_POSITION = 0.05; // Replace with actual extended position
private static final double UPWARDS_VOLTAGE = 3;
private static final double DOWNWARDS_VOLTAGE = -0.5;
private static final double kP = 0;
private static final double kI = 0.0;
private static final double kD = 0.0;
private static final double kS = 0.146;
private static final double kV = 0;
private static final double kA = 0.0;
private static final double kG = 0.606;
private static final double MECHANISM_RATIO = 7/1;
private static final double POSITION_TOLERANCE = 0.05;

private final Timer staticTimer = new Timer();
private final double STATIC_TIME_SECS = 0.7;
private final double minVelocityThresh = 0.5;

private AlgaePivotStates lastGoal = null;
private boolean slammed = false;

private TalonFX m_algaePivotMaster;
private TalonFX m_algaePivotSlave;

private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
private VoltageOut voltageOut = new VoltageOut(0);
private NeutralModeValue currentNeutralMode = NeutralModeValue.Brake;

public static boolean isAlgaeIntaked = false;

public AlgaePivot() {
    setName("AlgaeIntake");
    m_algaePivotMaster = new TalonFX(ALGAE_PIVOT_MASTER_ID, "rio");
    m_algaePivotSlave = new TalonFX(ALGAE_PIVOT_SLAVE_ID, "rio");
    configureMotor();
}

private void configureMotor() {
    TalonFXConfigurator.configureTalonFX(
        m_algaePivotMaster,
        "KrakenX60",
        currentNeutralMode,
        InvertedValue.Clockwise_Positive,
        GravityTypeValue.Arm_Cosine, kP, kI, kD, kS, kV, kA, kG,
        MECHANISM_RATIO,
        null, null, null
    );

    TalonFXConfigurator.configureTalonFX(
        m_algaePivotSlave,
        "KrakenX60",
        currentNeutralMode,
        InvertedValue.CounterClockwise_Positive, 
        null, null, null, null, null, null, null, null, null, null, null, null
    );
    
    Follower followerConfig = new Follower(m_algaePivotMaster.getDeviceID(), true);
        m_algaePivotSlave.setControl(followerConfig);
}

@Override
protected void executeCurrentStateBehavior() {
    //6328 GenericSlamElevator 
}

private void setMotorPosition(double position) {
    if (m_algaePivotMaster.isAlive()) {
        m_algaePivotMaster.setControl(positionVoltage.withPosition(position));
    }
}

private void setMotorVoltage(double volts){
    if (m_algaePivotMaster.isAlive()) {
        m_algaePivotMaster.setControl(voltageOut.withOutput(volts));
    }
}

public boolean atGoal() {
    AlgaePivotStates desiredState = (AlgaePivotStates) getDesiredState();
    double currentPosition = m_algaePivotMaster.getPosition().getValueAsDouble();
    double targetPosition = desiredState.getSetpointValue();
    
    return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
}

private void resetAlgaePivotPosition() {
    m_algaePivotMaster.setPosition(0);
}

private void toggleIdleMode(){
    currentNeutralMode = (currentNeutralMode == NeutralModeValue.Brake) 
        ? NeutralModeValue.Coast 
        : NeutralModeValue.Brake;
    configureMotor();
}

public void zeroAndToggleIdleMode() {
    resetAlgaePivotPosition();
    toggleIdleMode();
}

private final SysIdRoutine algaePivotCharacterization = new SysIdRoutine(
    new SysIdRoutine.Config(
        Volts.of(1).per(Second), 
        Volts.of(4.3), 
        Time.ofBaseUnits(6, Seconds),
        (state)-> SignalLogger.writeString("AlgaePivotState", state.toString())),
    new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
        m_algaePivotMaster.setControl(voltageOut.withOutput(volts));
        },
        null,
        this
    )
);

public Command algaePivotSysIDQuasistatic(SysIdRoutine.Direction direction) {
    return algaePivotCharacterization.quasistatic(direction);
}

public Command algaePivotSysIDDynamic(SysIdRoutine.Direction direction) {
    return algaePivotCharacterization.dynamic(direction);
}

private double getVelocityRads() {
    return (m_algaePivotMaster.getVelocity().getValueAsDouble())*Math.PI*2;
}

@Override
public void stop() {
    m_algaePivotMaster.stopMotor();
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
    return AlgaePivotStates.values();
}

@Override
public Enum<?> getDesiredState() {
    return desiredState;
}

@Override
public void periodic() { 
    // Call the FSMSubsystem's update method
    update();
    AlgaePivotStates newState = (AlgaePivotStates)getCurrentState();
    
    if (newState != lastGoal) {
        slammed = false;
        staticTimer.stop();
        staticTimer.reset();
    }

    lastGoal = newState;

    if (!slammed) {
        // Start static timer if within min velocity threshold.
        if (Math.abs(getVelocityRads()) <= minVelocityThresh) {
          staticTimer.start();
        } else {
          staticTimer.stop();
          staticTimer.reset();
        }
        // If we are finished with timer, finish goal.
        // Also assume we are at the goal if auto was started
        slammed = staticTimer.hasElapsed(STATIC_TIME_SECS) || DriverStation.isAutonomousEnabled();
      } else {
        staticTimer.stop();
        staticTimer.reset();
      }
    
    if (!slammed) {
        setMotorVoltage(newState.getSetpointValue());
        } else {
            stop();
    }

    if (DriverStation.isDisabled()){
        lastGoal = null;
        staticTimer.stop();
        staticTimer.reset();
        if (Math.abs(getVelocityRads())>minVelocityThresh){
            slammed = false;
        }
    }
    SmartDashboard.putString("Algae Pivot State", getCurrentState().toString());
    SmartDashboard.putNumber("Algae Pivot Position", m_algaePivotMaster.getPosition().getValueAsDouble());
    SmartDashboard.putString("Algae Pivot NeutralMode", currentNeutralMode.toString());
    SmartDashboard.putBoolean("Alage Pivot Slammed?", slammed);
    SmartDashboard.putNumber("Static Timer Elapsed", staticTimer.get());
    SmartDashboard.putNumber("Algae Pivot rads", Math.abs(getVelocityRads()));
}

public enum AlgaePivotStates {
    DOCKED(UPWARDS_VOLTAGE),
    INTAKE(DOWNWARDS_VOLTAGE);

    private final double setpointValue;

    AlgaePivotStates(double setpointValue) {
        this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
        return setpointValue;
    }
}
}
