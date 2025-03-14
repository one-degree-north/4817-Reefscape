// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FSMSubsystem extends SubsystemBase {
    protected Enum<?> currentState = null;
    protected Enum<?> desiredState = null;
    private boolean statesRegistered = false;

    // private boolean waitingForCondition = false;
    // private BooleanSupplier transitionCondition = null;

    public void registerState(Enum<?> initialState) {
        if (!statesRegistered) {
            this.currentState = initialState;
            this.desiredState = initialState;
            statesRegistered = true;
        }
    }

    public void update() {
        if (!statesRegistered) {
            return; // Do nothing if states are not registered yet
        } else if (currentState != desiredState) {
            handleStateTransition();
            executeCurrentStateBehavior();
            
        }
        
    }

    @Deprecated
    // Method to set the desired state with a condition
    public void setGoalWithCondition(Enum<?> newState, BooleanSupplier condition) {
        this.desiredState = newState;
        // this.waitingForCondition = true;
        // this.transitionCondition = condition;
    }


    // Method to set the desired state
    public void setGoal(Enum<?> newState) {
        this.desiredState = newState;
    }

    // Handle the transition between states
    private void handleStateTransition() {
        // exitCurrentState();
        currentState = desiredState;
        // enterNewState();
    }

    // Abstract methods that must be implemented by inheriting subsystems
    // protected abstract void enterNewState();
    // protected abstract void exitCurrentState();

    protected abstract void executeCurrentStateBehavior();
    public abstract boolean isInState(Enum<?> state);
    public abstract Enum<?> getCurrentState();
    public abstract Enum<?> getDesiredState();
    public abstract void stop();

    // Abstract method to define states
    protected abstract Enum<?>[] getStates();
}
