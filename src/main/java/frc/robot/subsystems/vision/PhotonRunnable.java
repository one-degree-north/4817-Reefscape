package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  // Array of photon pose estimators for photon cameras
  private final PhotonPoseEstimator[] photonPoseEstimators;
  // Array of subscribers for camera subtables
  private final RawSubscriber[] rawBytesSubscribers;
  // Array of subscriber wait handles for camera subtables, used to lookup camera index from wait handle
  private final int[] waitHandles;
  // Array of PhotonCameras for reading results
  private final PhotonCamera[] photonCameras;

  // Consumer of pose estimates
  private final AddVisionMeasurement poseConsumer;

  private final Supplier<Pose2d> poseSupplier;

  //STD for Cams
  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  //Field Dimensions
  private static final Distance FIELD_LENGTH = Meters.of(17.548);
  private static final Distance FIELD_WIDTH = Meters.of(8.052);

  //Thresholds
  private static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  private static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.5);

  @SuppressWarnings("unchecked")
  private final StructArrayPublisher<Pose3d>[] aprilTagPublishers = new StructArrayPublisher[2];

  public PhotonRunnable(
      String[] cameraNames,
      Transform3d[] robotToCameras,
      AddVisionMeasurement poseConsumer,
      Supplier<Pose2d> poseSupplier) {
    this.poseConsumer = poseConsumer;
    this.poseSupplier = poseSupplier;

    // NT publishers to send data to AdvantageScope
    for (int i = 0; i < cameraNames.length; i++) {
      aprilTagPublishers[i] = NetworkTableInstance.getDefault()
          .getStructArrayTopic("AprilTags-" + cameraNames[i], Pose3d.struct)
          .publish();
    }

    rawBytesSubscribers = new RawSubscriber[cameraNames.length];
    photonCameras = new PhotonCamera[cameraNames.length];
    photonPoseEstimators = new PhotonPoseEstimator[cameraNames.length];
    waitHandles = new int[cameraNames.length];

    AprilTagFieldLayout aprilTagFieldLayout;
    aprilTagFieldLayout = AprilTagFieldLayout
          .loadField(AprilTagFields.k2025Reefscape);

      for (int i = 0; i < cameraNames.length; i++) {
        var cameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraNames[i]);
        rawBytesSubscribers[i] = cameraTable.getRawTopic("rawBytes")
            .subscribe(
                PhotonPipelineResult.photonStruct.getTypeString(),
                  new byte[] {},
                  PubSubOption.periodic(0.01),
                  PubSubOption.sendAll(true),
                  PubSubOption.pollStorage(20));
        waitHandles[i] = rawBytesSubscribers[i].getHandle();
        photonPoseEstimators[i] = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCameras[i]);
        photonCameras[i] = new PhotonCamera(cameraNames[i]);
      }
  }

  @Override
  public void run() {
    var emptyAprilTagArray = new Pose3d[0];
    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from PhotonVision
      int[] signaledHandles = null;
      try {
        signaledHandles = WPIUtilJNI.waitForObjects(waitHandles);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

      var currentRobotPose = poseSupplier.get();
      for (int i = 0; i < signaledHandles.length; i++) {
        int cameraIndex = getCameraIndex(signaledHandles[i]);
        var aprilTagPublisher = aprilTagPublishers[cameraIndex];
        var photonPoseEstimator = photonPoseEstimators[cameraIndex];
        var photonCamera = photonCameras[cameraIndex];

        // Get AprilTag data
        var photonResults = photonCamera.getAllUnreadResults();
        photonResults.forEach(photonResult -> {
          if (photonResult.hasTargets() && (photonResult.targets.size() > 1
              || (photonResult.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD))) {

            // Send the AprilTag(s) to NT for AdvantageScope
            aprilTagPublisher.accept(
                photonResult.targets.stream()
                    .map(
                        target -> getTargetPose(
                            target,
                              currentRobotPose,
                              photonPoseEstimator.getRobotToCameraTransform()))
                    .toArray(Pose3d[]::new));

            photonPoseEstimator.update(photonResult).ifPresent(estimatedRobotPose -> {
              var estimatedPose = estimatedRobotPose.estimatedPose;
              // Make sure the measurement is on the field
              if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH.in(Meters)
                  && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH.in(Meters)) {

                var stdDevs = getEstimationStdDevs(
                    estimatedPose.toPose2d(),
                      photonResult.getTargets(),
                      photonPoseEstimator.getFieldTags());
                poseConsumer
                    .addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, stdDevs);
              }
            });
          } else {
            // No tags, send empty array to NT
            aprilTagPublisher.accept(emptyAprilTagArray);
          }
        });
      }
    }
    Arrays.stream(rawBytesSubscribers).forEach(RawSubscriber::close);
    Arrays.stream(aprilTagPublishers).forEach(StructArrayPublisher::close);
  }

  /**
   * Transform a target from PhotonVision to a pose on the field
   *
   * @param target target data from PhotonVision
   * @param robotPose current pose of the robot
   * @param robotToCamera transform from robot to the camera that saw the target
   * @return an AprilTag with an ID and pose
   */
  private static Pose3d getTargetPose(PhotonTrackedTarget target, Pose2d robotPose, Transform3d robotToCamera) {
    return new Pose3d(robotPose).transformBy(robotToCamera).transformBy(target.getBestCameraToTarget());
  }

  /**
   * Find the camera index for a table wait handle
   *
   * @param signaledHandle handle
   * @return index, or -1 if not found
   */
  private int getCameraIndex(int signaledHandle) {
    for (int i = 0; i < waitHandles.length; i++) {
      if (waitHandles[i] == signaledHandle) {
        return i;
      }
    }
    return -1;
  }

  /**
   * Scales the standard deviation based on the number of targets and their distance.
   *
   * @param estimatedPose estimated pose
   * @param targets targets from PhotonVision
   * @param fieldLayout tag poses
   */
  private static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose,
      List<PhotonTrackedTarget> targets,
      AprilTagFieldLayout fieldLayout) {

    var estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
        var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
        continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevs;
    }

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
    return estStdDevs;
  }

  @FunctionalInterface
  public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}