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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.leds.LEDs;
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

  private final LEDs s_Leds = new LEDs();

  private final DigitalInput zeroSwitch = new DigitalInput(1);

  private final Trigger zeroSwitchTrigger;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private RobotMode currentMode = RobotMode.DRIVING;
  private TuningSubsystem currentTuningSubsystem = TuningSubsystem.ALGAEINTAKE;

  private final SendableChooser<Command> autoChooser;

  private SuperstructureStates elevatorState = SuperstructureStates.STOWED;
  private SuperstructureStates algaeRemovalState = SuperstructureStates.ALGAE_REMOVE_LVL2;
  private Supplier<SuperstructureStates> elevatorStateSupplier = () -> elevatorState;
  private Supplier<SuperstructureStates> algaeRemovalStateSupplier = () -> algaeRemovalState;
  //These  are used to set the elevator state based on the operator input

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    zeroSwitchTrigger = new Trigger(() -> zeroSwitch.get());
    autoChooser = AutoBuilder.buildAutoChooser("Default_Auto");
    visionCommand.schedule();
    registerStatesNamedCommands();
    switch(currentMode){
      case DRIVING:
        driverBindings();
        break;
      case TUNING:
        tuningBindings();
        break;
    }
    //operatorBindings();
  }

  private void registerStatesNamedCommands(){
    s_Elevator.registerState(ElevatorStates.DOCKED);
    s_AlgaeIntake.registerState(AlgaeIntakeStates.IDLE);
    s_CoralIntake.registerState(CoralIntakeStates.ROLLER_IDLE);
    s_AlgaeIndexer.registerState(AlgaeIndexerStates.IDLE);
    s_Superstructure.registerState(SuperstructureStates.STOWED);
    s_CoralPivot.registerState(CoralPivotStates.DOCKED);
    s_AlgaePivot.registerState(AlgaePivotStates.DOCKED);

    NamedCommands.registerCommand("L4 Score", 
      s_Superstructure.setGoalCommand(SuperstructureStates.CORAL_LVL4));
    NamedCommands.registerCommand("Outtake Coral", 
      s_CoralIntake.setGoalCommand(CoralIntakeStates.ROLLER_OUTTAKE));
  }

  private void driverBindings() {
    //ZeroSwitch Binding
    zeroSwitchTrigger.onTrue(
      Commands.parallel(
        Commands.runOnce(() -> s_Superstructure.zeroSuperstructure()),
        Commands.runOnce(() -> s_Leds.zeroed = true)
      )
    );

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
              .withVelocityY(-driver.getLeftX() * MaxSpeed)
              .withRotationalRate(-driver.getRightX() * MaxAngularRate)
      )
    );

    //CORAL SCORING
    driver.R2().whileTrue(
      Commands.deadline(
        s_Superstructure.setConditionalGoalCommand(elevatorStateSupplier), 
        Commands.sequence( 
          Commands.waitUntil(()-> driver.getR2Axis() > 0.9),
          s_CoralIntake.setGoalCommand(CoralIntakeStates.ROLLER_OUTTAKE)
        )
      )
    );

    //ALGAE SHOOT
    driver.L2().whileTrue(
      Commands.sequence(
        Commands.deadline(
          Commands.waitUntil(() -> driver.getL2Axis() > 0.9),
          s_Superstructure.setGoalCommand(SuperstructureStates.ALGAE_EXTENDED),
          s_AlgaeIntake.setGoalCommand(AlgaeIntakeStates.SHOOT),
          s_AlgaeIndexer.setGoalCommand(AlgaeIndexerStates.INTAKING)),
        Commands.parallel(
          s_AlgaeIndexer.setGoalCommand(AlgaeIndexerStates.OUTTAKING),
          s_AlgaeIntake.setGoalCommand(AlgaeIntakeStates.SHOOT) 
        )
      )
    );

    //REMOVE ALGAE FROM REEF
    driver.R1().whileTrue(
      Commands.parallel(
        s_Superstructure.setConditionalGoalCommand(algaeRemovalStateSupplier),
        s_CoralIntake.setGoalCommand(CoralIntakeStates.ROLLER_INTAKE)
      )
    );

    //OUTTAKE CORAL
    driver.circle().whileTrue(
      Commands.parallel(
        //s_Superstructure.setGoalCommand(SuperstructureStates.CORAL_HP),
        s_CoralIntake.setGoalCommand(CoralIntakeStates.ROLLER_OUTTAKE)
      )
    );

    //INTAKE CORAL FROM HP
    driver.triangle().whileTrue(
      Commands.parallel(
        //s_Superstructure.setGoalCommand(SuperstructureStates.CORAL_HP),
        s_CoralIntake.setGoalCommand(CoralIntakeStates.ROLLER_INTAKE)
      )
    );

    //SHOOT ALGAE (NO PIVOT)
    driver.cross().whileTrue(
      Commands.sequence(
        Commands.deadline(
          Commands.waitSeconds(2),
          s_AlgaeIntake.setGoalCommand(AlgaeIntakeStates.SHOOT),
          s_AlgaeIndexer.setGoalCommand(AlgaeIndexerStates.INTAKING)
        ),
        Commands.parallel(
          s_AlgaeIndexer.setGoalCommand(AlgaeIndexerStates.OUTTAKING),
          s_AlgaeIntake.setGoalCommand(AlgaeIntakeStates.SHOOT)
        )
        
      )
    );

    //INTAKE ALGAE
    driver.square().whileTrue(
      Commands.parallel(
        //s_Superstructure.setGoalCommand(SuperstructureStates.ALGAE_EXTENDED),
        s_AlgaeIndexer.setGoalCommand(AlgaeIndexerStates.INTAKING),
        s_AlgaeIntake.setGoalCommand(AlgaeIntakeStates.INTAKE)
      )
    );
  }

  private void operatorBindings(){
    // Bind operator buttons to change the elevatorState variable
    operator.circle().onTrue(
      Commands.runOnce(() -> {
        elevatorState = SuperstructureStates.CORAL_LVL1; // Set to Level 1
    }));

    operator.cross().onTrue(
      Commands.runOnce(() -> {
        elevatorState = SuperstructureStates.CORAL_LVL2; // Set to Level 2
        algaeRemovalState = SuperstructureStates.ALGAE_REMOVE_LVL2;
    }));

    operator.triangle().onTrue(
      Commands.runOnce(() -> {
        elevatorState = SuperstructureStates.CORAL_LVL3; // Set to Level 3
        algaeRemovalState = SuperstructureStates.ALGAE_REMOVE_LVL3;
    }));

    operator.square().onTrue(
      Commands.runOnce(() -> {
        elevatorState = SuperstructureStates.CORAL_LVL4; // Set to Level 4
    }));
  }

  private void tuningBindings() {
    switch (currentTuningSubsystem) {
      case SWERVE: //SWITCH BETWEEN TRANSLATION, ROTATION, and STEER inside CommandSwerveDrivetrain
        driver.triangle().whileTrue(
          drivetrain.sysIdDynamic(Direction.kForward));
        driver.circle().whileTrue(
          drivetrain.sysIdDynamic(Direction.kReverse));
        driver.square().whileTrue(
          drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.cross().whileTrue(
          drivetrain.sysIdQuasistatic(Direction.kReverse));
        break;
      case ELEVATOR:
        driver.triangle().whileTrue(
          s_Elevator.elevatorSysIDDynamic(Direction.kForward));
        driver.circle().whileTrue(
          s_Elevator.elevatorSysIDDynamic(Direction.kReverse));
        driver.square().whileTrue(
          s_Elevator.elevatorSysIDQuasistatic(Direction.kForward));
        driver.cross().whileTrue(
          s_Elevator.elevatorSysIDQuasistatic(Direction.kReverse));
        break;
      case CORALPIVOT:
        driver.triangle().whileTrue(
          s_CoralPivot.coralPivotSysIDDynamic(Direction.kForward));
        driver.circle().whileTrue(
          s_CoralPivot.coralPivotSysIDDynamic(Direction.kReverse));
        driver.square().whileTrue(
          s_CoralPivot.coralPivotSysIDQuasistatic(Direction.kForward));
        driver.cross().whileTrue(
          s_CoralPivot.coralPivotSysIDQuasistatic(Direction.kReverse));
        break;
      case ALGAEPIVOT:
        driver.triangle().whileTrue(
          s_AlgaePivot.algaePivotSysIDDynamic(Direction.kForward));
        driver.circle().whileTrue(
          s_AlgaePivot.algaePivotSysIDDynamic(Direction.kReverse));
        driver.square().whileTrue(
          s_AlgaePivot.algaePivotSysIDQuasistatic(Direction.kForward));
        driver.cross().whileTrue(
          s_AlgaePivot.algaePivotSysIDQuasistatic(Direction.kReverse));
        break;
      case ALGAEINTAKE:
        driver.triangle().whileTrue(
          s_AlgaeIntake.elevatorSysIDDynamic(Direction.kForward));
        driver.circle().whileTrue(
          s_AlgaeIntake.elevatorSysIDDynamic(Direction.kReverse));
        driver.square().whileTrue(
          s_AlgaeIntake.elevatorSysIDQuasistatic(Direction.kForward));
        driver.cross().whileTrue(
          s_AlgaeIntake.elevatorSysIDQuasistatic(Direction.kReverse));
        break;
    }
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  private enum RobotMode {
    DRIVING, TUNING
  }

  private enum TuningSubsystem {
    SWERVE, ELEVATOR, CORALPIVOT, ALGAEPIVOT, ALGAEINTAKE
  }
}
