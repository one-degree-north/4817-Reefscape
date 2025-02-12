// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.FSMSubsystem;

public class Superstructure extends FSMSubsystem {

    private final AlgaePivot s_algaePivot;
    private final CoralPivot s_coralPivot;
    private final Elevator s_elevator;

    public Superstructure(AlgaePivot algaePivot, CoralPivot coralPivot, Elevator elevator) {
        setName("Superstructure");
        s_algaePivot = algaePivot;
        s_coralPivot = coralPivot;
        s_elevator = elevator;
    }

    @Override
    protected void enterNewState() {
        SuperstructureStates newState = (SuperstructureStates) getDesiredState();
        switch (newState) {
            case ALGAE_EXTENDED:
                s_algaePivot.setDesiredStateWithCondition(AlgaePivot.AlgaeStates.INTAKING, s_algaePivot::isAtSetpoint);
                break;
            case ALGAE_DOCKED:
                s_algaePivot.setDesiredStateWithCondition(AlgaePivot.AlgaeStates.DOCKED, s_algaePivot::isAtSetpoint);
                break;
            case CORAL_LVL1:
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_REEF, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_L1, s_elevator::isElevatorAtGoal);
                break;
            case CORAL_LVL2:
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_REEF, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_L2, s_elevator::isElevatorAtGoal);
                break;
            case CORAL_LVL3:
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_REEF, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_L3, s_elevator::isElevatorAtGoal);
                break;
            case CORAL_HP:
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_HUMAN_PLAYER, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_HP, s_elevator::isElevatorAtGoal);
                break;
            case CORAL_LVL4:
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_REEF, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_L4, s_elevator::isElevatorAtGoal);
                break;
            case FULLY_DOCKED:
                s_algaePivot.setDesiredStateWithCondition(AlgaePivot.AlgaeStates.DOCKED, s_algaePivot::isAtSetpoint);
                s_coralPivot.setDesiredStateWithCondition(CoralPivot.CoralPivotStates.CORAL_PIVOT_DOCKED, s_coralPivot::isAtGoal);
                s_elevator.setDesiredStateWithCondition(Elevator.ElevatorStates.ELEVATOR_DOCKED, s_elevator::isElevatorAtGoal);
                break;
        }
    }

    @Override
    protected void exitCurrentState() {
        // No specific exit actions needed
    }

    @Override
    protected void executeCurrentStateBehavior() {
        // Check if all subsystems have reached their desired states
        if (s_algaePivot.isAtSetpoint() && s_coralPivot.isAtGoal() && s_elevator.isElevatorAtGoal()) {
            setDesiredState(getCurrentState()); // Transition to the current state
        }
    }

    @Override
    public void stop() {
        s_algaePivot.stop();
        s_coralPivot.stop();
        s_elevator.stop();
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
        return SuperstructureStates.values();
    }

    @Override
    public void periodic() {
        update(); // Call the FSMSubsystem's update method
        SmartDashboard.putString("SuperstructureState", getCurrentState().toString());
    }

    public enum SuperstructureStates {
        ALGAE_EXTENDED, ALGAE_DOCKED,
        CORAL_LVL1, CORAL_LVL2, CORAL_LVL3,
        CORAL_HP, CORAL_LVL4, FULLY_DOCKED
    }
}
