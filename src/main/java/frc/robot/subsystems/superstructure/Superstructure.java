// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command setGoalCommand(SuperstructureStates goal) {
        return startEnd(()-> setGoal(goal), ()-> setGoal(SuperstructureStates.STOWED));
    }

    public Command setConditionalGoalCommand(Supplier<SuperstructureStates> goal) {
        return setGoalCommand(goal.get());
    }

    public Command zeroSuperstructure(){
        return Commands.parallel(
            Commands.runOnce(()-> s_algaePivot.zeroAndToggleIdleMode(), s_algaePivot),
            Commands.runOnce(()-> s_coralPivot.zeroAndToggleIdleMode(), s_coralPivot),
            Commands.runOnce(()-> s_elevator.zeroAndToggleIdleMode(), s_elevator),
            Commands.print("Zeroing Superstructure")
        );
    }

    @Override
    protected void executeCurrentStateBehavior() {
        SuperstructureStates newState = (SuperstructureStates) getCurrentState();
        switch (newState) {
            case ALGAE_EXTENDED:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.INTAKE);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.DOCKED);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.DOCKED);
                break;
            case CORAL_LVL1:
                s_algaePivot
            .       setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L1);
                break;
            case CORAL_LVL2:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L2);
                break;
            case CORAL_LVL3:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L3);
                break;
            case CORAL_HP:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.HUMAN_PLAYER);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.HUMAN_PLAYER);
                break;
            case CORAL_LVL4:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L4);
                break;
            case STOWED:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.DOCKED);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.DOCKED);
                break;
            case ALGAE_REMOVE_LVL2:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L2);
                break;
            case ALGAE_REMOVE_LVL3:
                s_algaePivot
                    .setGoal(
                        AlgaePivot.AlgaePivotStates.DOCKED);
                s_coralPivot
                    .setGoal(
                        CoralPivot.CoralPivotStates.REEF);
                s_elevator
                    .setGoal(
                        Elevator.ElevatorStates.L3);
                break;
            default:
                break;
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
        SmartDashboard.putString("Superstructure State", getCurrentState().toString());
    }

    public enum SuperstructureStates {
        ALGAE_EXTENDED, ALGAE_DOCKED,
        CORAL_LVL1, CORAL_LVL2, CORAL_LVL3,
        CORAL_HP, CORAL_LVL4, ALGAE_REMOVE_LVL2, 
        ALGAE_REMOVE_LVL3, STOWED
    }
}
