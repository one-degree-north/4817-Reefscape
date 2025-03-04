// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.algaerollers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.FSMSubsystem;
import frc.utils.TalonFXConfigurator;

public class AlgaeIndexer extends FSMSubsystem {
    // Constants
    private static final int INDEXER_MOTOR_ID = 43; // Replace with actual ID
    //private static final int BEAM_BREAK_PORT = 0; // Replace with actual port
    private static final double INTAKE_VOLTAGE = 1.0; // Replace with actual voltage
    private static final double OUTTAKE_VOLTAGE = -1.0; // Replace with actual voltage

    private TalonFX m_indexerMotor;
    //private DigitalInput m_beamBreak;
    private VoltageOut voltageOut = new VoltageOut(0);

    public AlgaeIndexer() {
        setName("AlgaeIndexer");
        m_indexerMotor = new TalonFX(INDEXER_MOTOR_ID, "rio");
        //m_beamBreak = new DigitalInput(BEAM_BREAK_PORT);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfigurator.configureTalonFX(
            m_indexerMotor,
            "Falcon500",
            NeutralModeValue.Brake,
            InvertedValue.Clockwise_Positive,
            null, null, null, null, null, null, null,
             null, 
             null, null, null
        );
    }

    @Override
    protected void enterNewState() {
        AlgaeIndexerStates newState = (AlgaeIndexerStates)getCurrentState();
        setMotorVoltage(newState.getSetpointValue());
    }

    @Override
    protected void exitCurrentState() {
        // No specific exit actions needed
    }

    @Override
    protected void executeCurrentStateBehavior() {
    }

    private void setMotorVoltage(double voltage) {
        if (m_indexerMotor.isAlive()) {
            m_indexerMotor.setControl(voltageOut.withOutput(voltage));
        }
    }

    // public boolean isBeamBreakActivated() {
    //     return m_beamBreak.get();
    // }

    public Command setGoalCommand(AlgaeIndexerStates goal) {
        return startEnd(()-> setGoal(goal), ()-> setGoal(AlgaeIndexerStates.IDLE));
    }

    @Override
    public void stop() {
        m_indexerMotor.stopMotor();
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
        return AlgaeIndexerStates.values();
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method
        SmartDashboard.putString("AlgaeIndexerState", getCurrentState().toString());
        //SmartDashboard.putBoolean("AlgaeBeamBreak", isBeamBreakActivated());
    }

    public enum AlgaeIndexerStates {
        INTAKING(INTAKE_VOLTAGE),
        IDLE(0),
        OUTTAKING(OUTTAKE_VOLTAGE);

        private final double setpointValue;

        AlgaeIndexerStates(double setpointValue) {
            this.setpointValue = setpointValue;
        }

        public double getSetpointValue() {
            return setpointValue;
        }
    }
}
