// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera mainCam;
  PhotonPipelineResult pipelineResult;
  PhotonPoseEstimator visionPoseEstimator;
  EstimatedRobotPose currentEstimatedPose;
  public Vision() {
    mainCam = new PhotonCamera(NetworkTableInstance.getDefault(), "maincamera");
    visionPoseEstimator = new PhotonPoseEstimator(FieldConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAMERA); 
  }
  public Pose3d GetCurrentEstimatedPose(){
    return currentEstimatedPose.estimatedPose;
  }
  public Transform3d GetRobotToTagTransform(int tagId, Pose3d currentRobotPose) {
    return FieldConstants.FIELD_LAYOUT.getTagPose(tagId).get().minus(currentRobotPose);
  }  
  public void updatePoseEstimator(Optional<EstimatedRobotPose> poseOptional){
    visionPoseEstimator.setReferencePose(poseOptional.get().estimatedPose);
    currentEstimatedPose = visionPoseEstimator.update().get();
  }

  @Override
  public void periodic() {
    currentEstimatedPose = visionPoseEstimator.update(pipelineResult).get();
  }
}
