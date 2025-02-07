// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConsumer;

/**
 * Command to use PhotonVision to process pose estimates and pass them to a pose estimator
 */
public class PhotonVisionCommand extends Command {
  private final Vision visions;
  private final VisionConsumer visionConsumer;

  /**
   * Constructs a PhotonVisionCommand
   * 
   * @param consumer consumer to receive vision estimates
   */
  public PhotonVisionCommand(VisionConsumer consumer) {
    visions = new Vision();
    this.visionConsumer = consumer;
  }

  @Override
  public void execute() {
    var visionEst = visions.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
    // Change our trust in the measurement based on the tags we can see
        var estStdDevs = visions.getEstimationStdDevs();
        visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
}
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
