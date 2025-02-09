package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class CoralIntake extends FSMSubsystem {
  // Constants
  private static final int m_leftRollerID = 10; // Replace with actual ID
  private static final int m_rightRollerID = 11; // Replace with actual ID
  private static final double m_rollerIntakeVoltage = 6.0; // Replace with actual voltage
  private static final double m_rollerOuttakeVoltage = -6.0; // Replace with actual voltage
  private static final double m_rollerLvl1Voltage = 3.0; // Replace with actual voltage for LVL1

  private TalonFX m_leftRoller;
  private TalonFX m_rightRoller;
  private VoltageOut voltageOut = new VoltageOut(0);

  public static boolean isCoralIntaked = false;

  public CoralIntake() {
    setName("CoralIntake");
    m_leftRoller = new TalonFX(m_leftRollerID, "rio");
    m_rightRoller = new TalonFX(m_rightRollerID, "rio");
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfigurator.configureTalonFX(
        m_leftRoller,
        "Falcon500",
        NeutralModeValue.Coast,
        InvertedValue.Clockwise_Positive,
        null, null, null, null, null, null, null, null, null, null, null
    );
    TalonFXConfigurator.configureTalonFX(
        m_rightRoller,
        "Falcon500",
        NeutralModeValue.Coast,
        InvertedValue.Clockwise_Positive,
        null, null, null, null, null, null, null, null, null, null, null
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
  protected void enterNewState() {
    CoralStates newState = (CoralStates)getCurrentState();
    switch (newState) {
      case ROLLER_LVL1:
        setRollersVoltage(m_rollerLvl1Voltage, 0); // Only left roller moves
        break;
      default:
        setRollersVoltage(newState.getSetpointValue(), newState.getSetpointValue());
        break;
    }
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
    return CoralStates.values();
  }

  @Override
  public void periodic() {
    update(); // Call the FSMSubsystem's update method
    SmartDashboard.putString("CoralIntakeState", getCurrentState().toString());
  }

  public enum CoralStates {
    ROLLER_INTAKE(m_rollerIntakeVoltage),
    ROLLER_OUTTAKE(m_rollerOuttakeVoltage),
    ROLLER_LVL1(m_rollerLvl1Voltage); // New state for LVL1

    private final double setpointValue;

    CoralStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
  }
}
