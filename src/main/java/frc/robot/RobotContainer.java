// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);
  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  private void configureBindings() {
  }
public Command getAutonomousCommand() {
    return null;
}
}
