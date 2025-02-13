// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class AlgaePivot extends FSMSubsystem {
// Constants
private static final int ALGAE_PIVOT_ID = 20; // Replace with actual ID
private static final double DOCKED_POSITION = 0.0; // Replace with actual docked position
private static final double INTAKING_POSITION = 10.0; // Replace with actual extended position
private static final double kP = 0.1;
private static final double kI = 0.0;
private static final double kD = 0.0;
private static final double kS = 0.0;
private static final double kV = 0.12;
private static final double kA = 0.0;
private static final double kG = 0.0;
private static final double MECHANISM_RATIO = 1.0;
private static final double MM_ACCELERATION = 100.0;
private static final double MM_CRUISE_VELOCITY = 200.0;
private static final double MM_JERK = 1000.0;
private static final double POSITION_TOLERANCE = 0.1;

private TalonFX m_algaePivot;
private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
private NeutralModeValue currentNeutralMode = NeutralModeValue.Brake;

public static boolean isAlgaeIntaked = false;

public AlgaePivot() {
    setName("AlgaeIntake");
    m_algaePivot = new TalonFX(ALGAE_PIVOT_ID, "rio");
    configureMotor();
}

private void configureMotor() {
    TalonFXConfigurator.configureTalonFX(
        m_algaePivot,
        "KrakenX60",
        currentNeutralMode,
        InvertedValue.Clockwise_Positive,
        kP, kI, kD, kS, kV, kA, kG,
        MECHANISM_RATIO,
        MM_ACCELERATION,
        MM_CRUISE_VELOCITY,
        MM_JERK
    );
}

@Override
protected void enterNewState() {
    AlgaePivotStates newState = (AlgaePivotStates)getCurrentState();
    setMotorPosition(newState.getSetpointValue());
}

@Override
protected void exitCurrentState() {
    // No specific exit actions needed
}

@Override
protected void executeCurrentStateBehavior() {
    setGoal(getCurrentState());
}

private void setMotorPosition(double position) {
    if (m_algaePivot.isAlive()) {
        m_algaePivot.setControl(motionMagicVoltage.withPosition(position));
    }
}

public boolean atGoal() {
    AlgaePivotStates desiredState = (AlgaePivotStates) getDesiredState();
    double currentPosition = m_algaePivot.getPosition().getValueAsDouble();
    double targetPosition = desiredState.getSetpointValue();
    
    return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
}

private void toggleIdleMode(){
    currentNeutralMode = (currentNeutralMode == NeutralModeValue.Brake) 
        ? NeutralModeValue.Coast 
        : NeutralModeValue.Brake;
    configureMotor();
}

private final SysIdRoutine algaePivotCharacterization = new SysIdRoutine(
    new SysIdRoutine.Config(null, Voltage.ofBaseUnits(2, Volts), null),
    new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
        m_algaePivot.setVoltage(volts.in(Volt));
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

@Override
public void stop() {
    m_algaePivot.stopMotor();
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
    update(); // Call the FSMSubsystem's update method
    SmartDashboard.putString("AlgaeIntakeState", getCurrentState().toString());
    SmartDashboard.putNumber("AlgaeIntakePosition", m_algaePivot.getPosition().getValueAsDouble());
    SmartDashboard.putString("AlgaeIntakeNeutralMode", currentNeutralMode.toString());
}

public enum AlgaePivotStates {
    DOCKED(DOCKED_POSITION),
    INTAKING(INTAKING_POSITION);

    private final double setpointValue;

    AlgaePivotStates(double setpointValue) {
        this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
        return setpointValue;
    }
}
}
