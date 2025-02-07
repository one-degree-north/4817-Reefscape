// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class CoralPivot extends FSMSubsystem {
  /** Creates a new CoralPivot. */

  //Constants

  private static final int CORAL_PIVOT_ID = 0;
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.0;
  private static final double kV = 0.12;
  private static final double kA = 0.0;
  private static final double kG = 0.0;
  private static final double CORAL_PIVOT_GEAR_RATIO = 0;
  private static final double mmAcceleration = 0;
  private static final double mmCruiseVelocity = 0;
  private static final double mmJerk = 0;
  private static final double CORAL_PIVOT_DOCKED_POS = 0.0;
  private static final double CORAL_PIVOT_HUMAN_PLAYER_POS = 0.0;
  private static final double CORAL_PIVOT_REEF_POS = 0.0;
  private static final double CORAL_PIVOT_ALLOWED_ERROR = 0;

  private TalonFX m_coralPivotMotor;

  private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    
  private NeutralModeValue m_currentNeutralMode = NeutralModeValue.Brake; //what is this

  public CoralPivot() {
    setName("Coral Pivot");
    m_coralPivotMotor = new TalonFX(CORAL_PIVOT_ID, "rio");
    motorConfigurations();
  }

  private void motorConfigurations() {
    TalonFXConfigurator.configureTalonFX(
      m_coralPivotMotor,
      "KrakenX60",
      m_currentNeutralMode,
      InvertedValue.Clockwise_Positive, //CHECK
      kP, kI, kD, kS, kV, kA, kG,
      CORAL_PIVOT_GEAR_RATIO,
      mmAcceleration,
      mmCruiseVelocity,
      mmJerk
      );
  }

  private void setControl(TalonFX motor, ControlRequest req) {
      if (motor.isAlive()) {
        motor.setControl(req);
      }
  }

  public void resetCoralPivotPosition() {
    m_coralPivotMotor.setPosition(0);
  }

  private void toggleIdleMode(){
    m_currentNeutralMode = (m_currentNeutralMode == NeutralModeValue.Brake) 
        ? NeutralModeValue.Coast 
        : NeutralModeValue.Brake;
    motorConfigurations(); //why are you configuring motor here
    SmartDashboard.putString("Coral Pivot Idle Mode", m_currentNeutralMode.toString());
  }

  private final SysIdRoutine elevatorCharacterization = new SysIdRoutine(
    new SysIdRoutine.Config(null, Voltage.ofBaseUnits(3, Volt), null),
    new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
            m_coralPivotMotor.setVoltage(volts.in(Volt));
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

  public boolean isCoralPivotAtGoal() {
    return Math.abs(m_coralPivotMotor.getPosition().getValueAsDouble() - 
        ((CoralPivotStates)getCurrentState()).getSetpointValue()) < CORAL_PIVOT_ALLOWED_ERROR;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void enterNewState() {
    CoralPivotStates newState = (CoralPivotStates)getCurrentState();
    setControl(m_coralPivotMotor, m_motionMagicVoltage.withPosition(newState.getSetpointValue()));
  }

  @Override
  protected void exitCurrentState() {
    // is there anything i need to add for this
    throw new UnsupportedOperationException("Unimplemented method 'exitCurrentState'");
  }



  @Override
  protected void executeCurrentStateBehavior() {
    // is there anything i need to add for this
    throw new UnsupportedOperationException("Unimplemented method 'executeCurrentStateBehavior'");
  }



  @Override
  public void stop() {
    m_coralPivotMotor.stopMotor();
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
    return CoralPivotStates.values();
  }

  public enum CoralPivotStates {
    CORAL_PIVOT_DOCKED(CORAL_PIVOT_DOCKED_POS),
    CORAL_PIVOT_HUMAN_PLAYER(CORAL_PIVOT_HUMAN_PLAYER_POS),
    CORAL_PIVOT_REEF(CORAL_PIVOT_REEF_POS);

    private final double setpointValue;

    CoralPivotStates(double setpointValue) {
        this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
        return setpointValue;
    }
}
}
