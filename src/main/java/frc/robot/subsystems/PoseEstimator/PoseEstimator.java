// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimator;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TunerConstants;

public class PoseEstimator extends SubsystemBase {
  //Both are matrixes are in the form [x, y, theta], increase numbers to trust vision/odometry more
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 1.8);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  private boolean tagSeenSinceLastDisable = false;

  private PhotonRunnable camera;
  private Notifier cameraNotifier;

  public PoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier, PhotonRunnable camera){
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;
    this.camera = camera;

    cameraNotifier = new Notifier(camera);
    cameraNotifier.setName("Winston4817");
    cameraNotifier.startPeriodic(0.02);

    poseEstimator =  new SwerveDrivePoseEstimator(
        TunerConstants.swerveKinematics,
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
  }
  


  @Override
  public void periodic() {
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

    var visionPose = camera.grabLatestEstimatedPose();
    if (visionPose != null) {
        if (!DriverStation.isEnabled()) {
            tagSeenSinceLastDisable = true;
        }

        var pose2d = visionPose.estimatedPose.toPose2d();
        Vector<N3> dynamicVisionMeasurementStdDevs = visionMeasurementStdDevs;

        if (camera.getTrackedVisionTargets().length > 0) {
            Pose3d closestTarget = camera.getTrackedVisionTargets()[0];
            for (Pose3d target : camera.getTrackedVisionTargets()) {
                if (target.toPose2d().getTranslation()
                        .getDistance(pose2d.getTranslation()) <
                    closestTarget.toPose2d().getTranslation()
                        .getDistance(pose2d.getTranslation())) {
                    closestTarget = target;
                }
            }

            double scalingFactor = Math.max(
                1.0, // Avoid division by zero
                closestTarget.toPose2d().getTranslation().getDistance(pose2d.getTranslation())
            ) * (1.0 / camera.getTrackedVisionTargets().length);

            dynamicVisionMeasurementStdDevs = visionMeasurementStdDevs.times(scalingFactor);
        }

        poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds, dynamicVisionMeasurementStdDevs);
    }

    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putString("Pose", getFormattedPose());
    SmartDashboard.putData("Pose Estimator Field2d", field2d);

    if (tagSeenSinceLastDisable && DriverStation.isEnabled()) {
        tagSeenSinceLastDisable = false;
    }
}

  @Logged
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Logged
  public Pose2d getVisionEstimatedPose(){
    if (camera.getTrackedVisionTargets().length > 0){
      return camera.getTrackedVisionTargets()[0].toPose2d();
    } else {
      //workaround for when no vision targets are seen (null values make things dissapear in advantagescope)
      return new Pose2d(1000, 1000, new Rotation2d());
    }
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}
