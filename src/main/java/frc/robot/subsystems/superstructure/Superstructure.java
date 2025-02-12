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
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.INTAKING);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.DOCKED);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.DOCKED);
                break;
            case ALGAE_DOCKED:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.DOCKED);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.DOCKED);
                break;
            case CORAL_LVL1:
                s_algaePivot
            .       setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.L1);
                break;
            case CORAL_LVL2:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.L2);
                break;
            case CORAL_LVL3:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.L3);
                break;
            case CORAL_HP:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.HUMAN_PLAYER);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.HUMAN_PLAYER);
                break;
            case CORAL_LVL4:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.L4);
                break;
            case STOWED:
                s_algaePivot
                    .setDesiredState(
                        AlgaePivot.AlgaeStates.DOCKED);
                s_coralPivot
                    .setDesiredState(
                        CoralPivot.CoralPivotStates.DOCKED);
                s_elevator
                    .setDesiredState(
                        Elevator.ElevatorStates.DOCKED);
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
        if (s_algaePivot.atGoal() && s_coralPivot.atGoal() && s_elevator.atGoal()) {
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
        CORAL_HP, CORAL_LVL4, STOWED
    }
}
