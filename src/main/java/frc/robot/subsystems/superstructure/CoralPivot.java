// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.FSMSubsystem;

public class CoralPivot extends FSMSubsystem {
  /** Creates a new CoralPivot. */
  public CoralPivot() {

  }

  private void toggleIdleMode(){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void enterNewState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'enterNewState'");
  }

  @Override
  protected void exitCurrentState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'exitCurrentState'");
  }



  @Override
  protected void executeCurrentStateBehavior() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'executeCurrentStateBehavior'");
  }



  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }



  @Override
  public boolean isInState(Enum<?> state) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isInState'");
  }



  @Override
  public Enum<?> getCurrentState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentState'");
  }



  @Override
  protected Enum<?>[] getStates() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getStates'");
  }
}
