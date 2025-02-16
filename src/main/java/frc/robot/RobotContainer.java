// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDStates;
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

  private final DigitalInput zeroSwitch = new DigitalInput(0);

  private final BooleanSupplier zeroSwitchSupplier = () -> zeroSwitch.get();
  private final BooleanSupplier autonSupplier = () -> DriverStation.isAutonomousEnabled();

  private final Trigger zeroSwitchTrigger;
  private final Trigger autonTrigger;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private RobotMode currentMode = RobotMode.DRIVING;
  private TuningSubsystem currentTuningSubsystem = TuningSubsystem.SWERVE;

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
    zeroSwitchTrigger = new Trigger(zeroSwitchSupplier);
    autonTrigger = new Trigger(autonSupplier);
    autoChooser = AutoBuilder.buildAutoChooser("Default_Auto");
    visionCommand.schedule();
    registerInitialStates();
    driverBindings();
    operatorBindings();
    
    driver.touchpad().onTrue(
      Commands.runOnce(() -> toggleRobotMode())
    );
  }

  private void registerInitialStates(){
    s_Leds.registerState(LEDStates.NOTZEROED);
    s_Elevator.registerState(ElevatorStates.DOCKED);
    s_AlgaeIntake.registerState(AlgaeIntakeStates.IDLE);
    s_CoralIntake.registerState(CoralIntakeStates.ROLLER_IDLE);
    s_AlgaeIndexer.registerState(AlgaeIndexerStates.INTAKED);
    s_Superstructure.registerState(SuperstructureStates.STOWED);
    s_CoralPivot.registerState(CoralPivotStates.DOCKED);
    s_AlgaePivot.registerState(AlgaePivotStates.DOCKED);
  }

  private void toggleRobotMode(){
    currentMode = (currentMode == RobotMode.DRIVING) ? RobotMode.TUNING : RobotMode.DRIVING;
    if (currentMode == RobotMode.TUNING) {
      tuningBindings();
    } else {
      driverBindings();
    }
  }

  private void driverBindings() {
    //ZeroSwitch Binding
    zeroSwitchTrigger.onTrue(
      Commands.parallel(
        Commands.runOnce(() -> s_Superstructure.zeroSuperstructure(), s_Superstructure),
        Commands.runOnce(() -> s_Leds.setGoal(LEDStates.ZEROED), s_Leds)
      )
    );

    autonTrigger.onTrue(
      Commands.runOnce(() -> s_Leds.setGoal(LEDStates.INTAKED_ALGAE), s_Leds)
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
          Commands.waitUntil(()-> driver.getR2Axis()<0.9),
          Commands.run(()-> s_CoralIntake.setGoal(CoralIntakeStates.ROLLER_OUTTAKE), s_CoralIntake)
        )
      )
    );

    //ALGAE SHOOT
    driver.L2().whileTrue(
      Commands.sequence(
        Commands.parallel(
          s_Superstructure.setGoalCommand(SuperstructureStates.ALGAE_EXTENDED),
          Commands.runOnce(()-> s_AlgaeIntake.setGoal(AlgaeIntakeStates.SHOOT), s_AlgaeIntake),
          Commands.runOnce(()-> s_AlgaeIndexer.setGoal(AlgaeIndexerStates.INTAKED), s_AlgaeIndexer)),
        Commands.waitUntil(() -> driver.getL2Axis() < 0.9),
        Commands.run(()-> s_AlgaeIndexer.setGoal(AlgaeIntakeStates.OUTTAKE), s_AlgaeIndexer)
      )
    );

    //REMOVE ALGAE FROM REEF
    driver.R1().whileTrue(
      Commands.parallel(
        s_Superstructure.setConditionalGoalCommand(algaeRemovalStateSupplier),
        Commands.run(()-> s_CoralIntake.setGoal(CoralIntakeStates.ROLLER_INTAKE), s_CoralIntake)
      )
    );

    //INTAKE CORAL FROM HP
    driver.triangle().whileTrue(
      Commands.parallel(
        s_Superstructure.setGoalCommand(SuperstructureStates.CORAL_HP),
        Commands.run(()-> s_CoralIntake.setGoal(CoralIntakeStates.ROLLER_INTAKE), s_CoralIntake)
      )
    );

    //INTAKE ALGAE
    driver.circle().whileTrue(
      Commands.parallel(
        s_Superstructure.setGoalCommand(SuperstructureStates.ALGAE_EXTENDED),
        Commands.run(()-> s_AlgaeIntake.setGoal(AlgaeIntakeStates.INTAKE), s_AlgaeIntake),
        Commands.run(()-> s_AlgaeIndexer.setGoal(AlgaeIndexerStates.INTAKING), s_AlgaeIndexer)
      )
    );
    //ALGAE: docked (both rollers not spining), intaking (both rollers spinning), ramping (pivot extended, end effector rolling), shoot(all motors spinning, pivot fully out), processor(pivot pulled back, stationary roller spinning)
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
