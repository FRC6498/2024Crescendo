// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision {
  /** Creates a new Vision. */
  PhotonCamera mainCam;
  PhotonPipelineResult pipelineResult;
  PhotonPoseEstimator visionPoseEstimator;
  EstimatedRobotPose currentEstimatedPose;
  Optional<EstimatedRobotPose> optionalPose; 

  VisionSystemSim visionSim;
  PhotonCameraSim mainCamSim;
  Supplier<Pose2d> simulatedRobotPose;

  public Vision(Supplier<Pose2d> simPose) {
    mainCam = new PhotonCamera(NetworkTableInstance.getDefault(), "maincamera");
    visionPoseEstimator = new PhotonPoseEstimator(FieldConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAMERA); 
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.FIELD_LAYOUT);
      mainCamSim = new PhotonCameraSim(mainCam, setupCameraProperties());
      visionSim.addCamera(mainCamSim, VisionConstants.ROBOT_TO_CAMERA);
      simulatedRobotPose = simPose;
      SmartDashboard.putData(visionSim.getDebugField());
    }
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

  public void periodic() {
    pipelineResult = mainCam.getLatestResult();
    currentEstimatedPose = visionPoseEstimator.update(pipelineResult).orElse(currentEstimatedPose);
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }
  }

  public void simulationPeriodic() {
    visionSim.update(simulatedRobotPose.get());
  }

  private SimCameraProperties setupCameraProperties() {
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(35);
    cameraProperties.setLatencyStdDevMs(5);
    return cameraProperties;
  }
}
