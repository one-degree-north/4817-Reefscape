package frc.robot.subsystems.rollers;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureStates;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class CoralIntake extends FSMSubsystem {
  // Constants
  private static final int LEFT_ROLLER_ID = 4; // Replace with actual ID
  private static final int RIGHT_ROLLER_ID = 5; // Replace with actual ID
  private static final double ROLLER_INTAKE_VOLTAGE = 4; // Replace with actual voltage
  private static final double ROLLER_OUTTAKE_VOLTAGE = -2.0; // Replace with actual voltage
  private static final double ROLLER_LVL1_VOLTAGE = -1; // Replace with actual voltage for LVL1

  private TalonFX m_leftRoller;
  private TalonFX m_rightRoller;
  private VoltageOut voltageOut = new VoltageOut(0);

  public static boolean isCoralIntaked = false;

  public CoralIntake() {
    setName("CoralIntake");
    m_leftRoller = new TalonFX(LEFT_ROLLER_ID, "rio");
    m_rightRoller = new TalonFX(RIGHT_ROLLER_ID, "rio");
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfigurator.configureTalonFX(
        m_leftRoller,
        "Falcon500",
        NeutralModeValue.Coast,
        InvertedValue.CounterClockwise_Positive,
        null, null, null, null, null, null, null, null, null, null, null, null
    );
    TalonFXConfigurator.configureTalonFX(
        m_rightRoller,
        "Falcon500",
        NeutralModeValue.Coast,
        InvertedValue.Clockwise_Positive,
        null, null, null, null, null, null, null, null, null, null, null, null
    );
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
  }

  private void stopMotor(TalonFX motor) {
    motor.stopMotor();
  }

  private void setRollersVoltage(double leftVoltage, double rightVoltage) {
    setControl(m_leftRoller, voltageOut.withOutput(leftVoltage));
    setControl(m_rightRoller, voltageOut.withOutput(rightVoltage));
  }

  @Override
  protected void executeCurrentStateBehavior() {
    CoralIntakeStates newState = (CoralIntakeStates)getCurrentState();
    switch (newState) {
      case ROLLER_LVL1:
        setRollersVoltage(ROLLER_LVL1_VOLTAGE, 0); // Only left roller moves
        break;
      default:
        setRollersVoltage(newState.getSetpointValue(), newState.getSetpointValue());
        break;
    }
  }

  @Override
  public void stop() {
    stopMotor(m_leftRoller);
    stopMotor(m_rightRoller);
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
    return CoralIntakeStates.values();
  }

  public Command setGoalCommand(CoralIntakeStates goal) {
    return startEnd(()-> setGoal(goal), ()-> setGoal(CoralIntakeStates.ROLLER_IDLE));
  }

  public Command setGoalCommandBasedOnSuperstructure(Supplier<SuperstructureStates> goal){
    var state = goal.get();
    if (state == SuperstructureStates.CORAL_LVL1){
      Commands.print("L1 Roller");
      return startEnd(()-> setGoal(CoralIntakeStates.ROLLER_LVL1), ()-> setGoal(CoralIntakeStates.ROLLER_IDLE));
    }
    else if (state == SuperstructureStates.CORAL_HP){
      Commands.print("Human Player Intaking");
      return startEnd(()-> setGoal(CoralIntakeStates.ROLLER_INTAKE), ()-> setGoal(CoralIntakeStates.ROLLER_IDLE));
    }
    else{
      Commands.print("Roller Outtaking");
      return startEnd(()-> setGoal(CoralIntakeStates.ROLLER_OUTTAKE), ()-> setGoal(CoralIntakeStates.ROLLER_IDLE));
    }
  }

  @Override
  public void periodic() {
    update(); // Call the FSMSubsystem's update method
    SmartDashboard.putString("Coral Intake State", getCurrentState().toString());
    SmartDashboard.putNumber("Coral Intake RPS", m_leftRoller.getVelocity().getValueAsDouble());
  }

  public enum CoralIntakeStates {
    ROLLER_INTAKE(ROLLER_INTAKE_VOLTAGE),
    ROLLER_OUTTAKE(ROLLER_OUTTAKE_VOLTAGE),
    ROLLER_IDLE(0),
    ROLLER_LVL1(ROLLER_LVL1_VOLTAGE); // New state for LVL1

    private final double setpointValue;

    CoralIntakeStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
  }
}
