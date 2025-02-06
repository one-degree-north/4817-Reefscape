// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimator;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

@Logged
public class PhotonRunnable implements Runnable{
  
  private final PhotonCamera m_Camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final AprilTagFieldLayout fieldLayout;  
  private ArrayList<Pose3d> trackedVisionTargets = new ArrayList<>();
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  

  public PhotonRunnable(Transform3d CamOffset) {
    m_Camera = new PhotonCamera("Winston4817");
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    photonPoseEstimator = new PhotonPoseEstimator(
      fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CamOffset);
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && m_Camera != null) {
      var photonResults = m_Camera.getLatestResult();

      // Consider targets underneath ambiguity threshold OR if there are multiple tags
      if (photonResults.hasTargets() 
          && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < 2)) 
      {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= fieldLayout.getFieldLength()
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= fieldLayout.getFieldWidth()) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });

        ArrayList<Pose3d> tempTrackedVisionTargets = new ArrayList<>();

        photonResults.getTargets().forEach(target -> {
          Optional<Pose3d> pose = fieldLayout.getTagPose(target.getFiducialId());
          if (pose.isPresent())
            tempTrackedVisionTargets.add(pose.get());
        });

        trackedVisionTargets = tempTrackedVisionTargets;

      }
    }   
  }
  
  public Pose3d[] getTrackedVisionTargets() {
    return trackedVisionTargets.toArray(new Pose3d[trackedVisionTargets.size()]);
  }

  /** 
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * this code does NOT handle alliance flipping.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }
}
