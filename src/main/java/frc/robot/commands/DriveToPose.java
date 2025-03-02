//Credits to 7028 for the original code

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

/**
 * Command to drive to a pose.
 */
public class DriveToPose extends Command {

    private static final double TRANSLATION_TOLERANCE = 0.02;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
    private final double THETA_kP = 0;
    private final double THETA_kI = 0;
    private final double THETA_kD = 0;
    private final double X_kP = 0;
    private final double X_kI = 0;
    private final double X_kD = 0;
    private final double Y_kP = 0;
    private final double Y_kI = 0;
    private final double Y_kD = 0;

    private static final double MAX_VELOCITY_METERS_PER_SECOND = 5.76;
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10;

    private final double FIELD_WIDTH_METERS = 8.0137;

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_VELOCITY_METERS_PER_SECOND * 0.5,
      MAX_VELOCITY_METERS_PER_SECOND);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public DriveToPose(
        CommandSwerveDrivetrain drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        boolean useAllianceColor) {
    this(drivetrainSubsystem, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
  }

  public DriveToPose(
        CommandSwerveDrivetrain drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        boolean useAllianceColor) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivetrainSubsystem);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    if (useAllianceColor && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    var applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.setControl(applyFieldSpeeds.withSpeeds(
        new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed)));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    var applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    drivetrainSubsystem.setControl(applyFieldSpeeds.withSpeeds(
        new ChassisSpeeds(0,0,0)));
  }

}