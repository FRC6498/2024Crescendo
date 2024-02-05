// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera mainCam;
  private PhotonPipelineResult pipelineResult;
  private final PhotonPoseEstimator visionPoseEstimator;
  public Optional<EstimatedRobotPose> currentEstimatedPose;
  public double lastTimestamp = 0;
  
  public boolean isBlueAlliance;
  public Vision() {
    mainCam = new PhotonCamera("mainCamera");
    visionPoseEstimator = new PhotonPoseEstimator(FieldConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,mainCam,  VisionConstants.ROBOT_TO_CAMERA); 
    visionPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }
  public Transform3d GetRobotToTagTransform(int tagId, Pose3d currentRobotPose) {
    return FieldConstants.FIELD_LAYOUT.getTagPose(tagId).get().minus(currentRobotPose);
  }  
  public Optional<EstimatedRobotPose> updatePoseEstimator(){
    var visionEstimate = visionPoseEstimator.update();
    double timestamp = mainCam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(timestamp - lastTimestamp) > 1e-5;
    if(newResult) lastTimestamp = timestamp;
    return visionEstimate;
  }
  public Transform3d GetRobotToSpeakerTransform() {
    if (currentEstimatedPose.isPresent()) {
      if (isBlueAlliance) {
      return GetRobotToTagTransform(FieldConstants.BLUE_SPEAKER_TAG_ID, currentEstimatedPose.get().estimatedPose);
    }else{
      return GetRobotToTagTransform(FieldConstants.RED_SPEAKER_TAG_ID, currentEstimatedPose.get().estimatedPose);
    }
    }else{
      return null;
    }
    
  }
  public double getCurrentTimeStamp() {
    return pipelineResult.getTimestampSeconds();
  }
  public double GetRobotToSpeakerDistance() {
    Transform3d robotToSpeakerTransform = GetRobotToSpeakerTransform();
    //basic pythagorean therom
    return Math.sqrt(Math.pow(robotToSpeakerTransform.getX(), 2) + Math.pow(robotToSpeakerTransform.getY(), 2));
  }

  @Override
  public void periodic() {
    if (mainCam.getLatestResult().hasTargets()) {
      pipelineResult = mainCam.getLatestResult();
      currentEstimatedPose = visionPoseEstimator.update(pipelineResult, mainCam.getCameraMatrix(), mainCam.getDistCoeffs());
    }
  }
}
