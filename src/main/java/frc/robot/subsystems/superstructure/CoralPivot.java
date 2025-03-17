// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class CoralPivot extends FSMSubsystem {
  //Constants

  private static final int CORAL_PIVOT_MASTER_ID = 3;
  private static final double kP = 29.729;
  private static final double kI = 0.0;
  private static final double kD = 1.5852;
  private static final double kS = 0.30161;
  private static final double kV = 2.847;
  private static final double kA = 0.084136;
  private static final double kG = 0.37407;
  private static final double MM_CRUISE_VELOCITY = 1;
  private static final double MM_ACCELERATION = 0.2;
  private static final double CORAL_PIVOT_GEAR_RATIO = 36/1;
  private static final double CORAL_PIVOT_DOCKED_POS = -0.15;
  private static final double CORAL_PIVOT_HUMAN_PLAYER_POS = 0.15;
  private static final double CORAL_PIVOT_REEF_POS = -0.15;
  private static final double CORAL_PIVOT_ALLOWED_ERROR = 0.05;

  private TalonFX m_coralPivot;

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);
    
  private NeutralModeValue m_currentNeutralMode = NeutralModeValue.Brake;

  public CoralPivot() {
    setName("Coral Pivot");
    m_coralPivot = new TalonFX(CORAL_PIVOT_MASTER_ID, "rio");
    motorConfigurations();
  }

  private void motorConfigurations() {
    TalonFXConfigurator.configureTalonFX(
      m_coralPivot,
      "Falcon500",
      m_currentNeutralMode,
      InvertedValue.Clockwise_Positive, //CHECK
      GravityTypeValue.Arm_Cosine, kP, kI, kD, kS, kV, kA, kG,
      CORAL_PIVOT_GEAR_RATIO,
      MM_ACCELERATION, MM_CRUISE_VELOCITY, null
      );
  }

  private void setControl(TalonFX motor, ControlRequest req) {
      if (motor.isAlive()) {
        motor.setControl(req);
      }
  }

  private void resetCoralPivotPosition() {
    m_coralPivot.setPosition(0);
  }

  private void toggleIdleMode(){
    m_currentNeutralMode = (m_currentNeutralMode == NeutralModeValue.Brake) 
        ? NeutralModeValue.Coast 
        : NeutralModeValue.Brake;
    motorConfigurations();
    SmartDashboard.putString("Coral Pivot Idle Mode", m_currentNeutralMode.toString());
  }

  public void zeroAndToggleIdleMode() {
    resetCoralPivotPosition();
    toggleIdleMode();
  }

  private final SysIdRoutine coralPivotCharacterization = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.of(0.7).per(Second), 
      Volt.of(1.1), 
      Time.ofBaseUnits(5, Seconds),
      (state)-> SignalLogger.writeString("CoralPivotState", state.toString())),
    new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
          m_coralPivot.setControl(voltageOut.withOutput(volts));
        },
        null,
        this
    )
  );
    
  public Command coralPivotSysIDQuasistatic(SysIdRoutine.Direction direction) {
    return coralPivotCharacterization.quasistatic(direction);
  }

  public Command coralPivotSysIDDynamic(SysIdRoutine.Direction direction) {
    return coralPivotCharacterization.dynamic(direction);
  }

  public boolean atGoal() {
    return Math.abs(m_coralPivot.getPosition().getValueAsDouble() - 
        ((CoralPivotStates)getDesiredState()).getSetpointValue()) < CORAL_PIVOT_ALLOWED_ERROR;
  }

  @Override
  protected void executeCurrentStateBehavior() {
    CoralPivotStates newState = (CoralPivotStates)getCurrentState();
    setControl(m_coralPivot, motionMagicVoltage.withPosition(newState.getSetpointValue()));
  }

  @Override
  public void stop() {
    m_coralPivot.stopMotor();
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
    return CoralPivotStates.values();
  }

  @Override
  public void periodic() {
    update();

    SmartDashboard.putString("Coral Pivot State", getCurrentState().toString());
    SmartDashboard.putNumber("Coral Pivot Position", m_coralPivot.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Coral Pivot AtGoal?", atGoal());
  }

  public enum CoralPivotStates {
    DOCKED(CORAL_PIVOT_DOCKED_POS),
    HUMAN_PLAYER(CORAL_PIVOT_HUMAN_PLAYER_POS),
    REEF(CORAL_PIVOT_REEF_POS);

    private final double setpointValue;

    CoralPivotStates(double setpointValue) {
        this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
        return setpointValue;
    }
}
}
