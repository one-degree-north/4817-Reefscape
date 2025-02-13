// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.rollers.CoralIntake;
import frc.robot.subsystems.rollers.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.rollers.algaerollers.AlgaeIndexer;
import frc.robot.subsystems.rollers.algaerollers.AlgaeIndexer.AlgaeIndexerStates;
import frc.robot.subsystems.rollers.algaerollers.AlgaeIntake;
import frc.robot.subsystems.rollers.algaerollers.AlgaeIntake.AlgaeIntakeStates;
import frc.robot.subsystems.superstructure.AlgaePivot;
import frc.robot.subsystems.superstructure.AlgaePivot.AlgaePivotStates;
import frc.robot.subsystems.superstructure.CoralPivot;
import frc.robot.subsystems.superstructure.CoralPivot.CoralPivotStates;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorStates;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureStates;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);
  private final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

  private final Elevator s_Elevator = new Elevator();
  private final CoralPivot s_CoralPivot = new CoralPivot();
  private final AlgaePivot s_AlgaePivot = new AlgaePivot();
  private final Superstructure s_Superstructure = new Superstructure(s_AlgaePivot, s_CoralPivot, s_Elevator);

  private final AlgaeIndexer s_AlgaeIndexer = new AlgaeIndexer();
  private final AlgaeIntake s_AlgaeIntake = new AlgaeIntake(false);
  private final CoralIntake s_CoralIntake = new CoralIntake();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private SuperstructureStates elevatorState = SuperstructureStates.STOWED;
  private Supplier<SuperstructureStates> elevatorStateSupplier = () -> elevatorState;
  //These two are used to set the elevator state based on the operator input

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Elevator.registerStates(ElevatorStates.DOCKED);
    s_AlgaeIntake.registerStates(AlgaeIntakeStates.IDLE);
    s_CoralIntake.registerStates(CoralIntakeStates.ROLLER_IDLE);
    s_AlgaeIndexer.registerStates(AlgaeIndexerStates.INTAKED);
    s_Superstructure.registerStates(SuperstructureStates.STOWED);
    s_CoralPivot.registerStates(CoralPivotStates.DOCKED);
    s_AlgaePivot.registerStates(AlgaePivotStates.DOCKED);
    visionCommand.schedule();
    driverBindings();
    operatorBindings();
  }

  private void driverBindings() {
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
    );

    driver.R2().whileTrue(
      Commands.deadline(
        s_Superstructure.setConditionalGoalCommand(elevatorStateSupplier), 
        Commands.sequence( 
          Commands.waitUntil(()-> driver.getR2Axis()<0.8),
          Commands.run(()-> s_CoralIntake.setGoal(CoralIntakeStates.ROLLER_OUTTAKE), s_CoralIntake)
        )
      )
    );
    //ALGAE: docked (both rollers not spining), intaking (both rollers spinning), ramping (pivot extended, end effector rolling), shoot(all motors spinning, pivot fully out), processor(pivot pulled back, stationary roller spinning)
  }

  private void operatorBindings(){
    // Bind operator buttons to change the elevatorState variable
    operator.circle().onTrue(Commands.runOnce(() -> {
      elevatorState = SuperstructureStates.CORAL_LVL1; // Set to Level 1
    }));

    operator.cross().onTrue(Commands.runOnce(() -> {
      elevatorState = SuperstructureStates.CORAL_LVL2; // Set to Level 2
    }));

    operator.triangle().onTrue(Commands.runOnce(() -> {
      elevatorState = SuperstructureStates.CORAL_LVL3; // Set to Level 3
    }));

    operator.square().onTrue(Commands.runOnce(() -> {
      elevatorState = SuperstructureStates.CORAL_LVL4; // Set to Level 4
    }));
  }
  public Command getAutonomousCommand() {
      return null;
  }
}
