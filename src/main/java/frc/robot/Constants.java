// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public final class Constants {
    public static final class VisionConstants{
       public static final Transform3d ROBOT_TO_CAMERA = new Transform3d();//TODO: Get the robot to camera transform
    }
    public static final class FieldConstants{
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final int BLUE_SPEAKER_TAG_ID = 4;//TODO:check if blue tag ids are right
        public static final int RED_SPEAKER_TAG_ID = 8; 
    }
    public static final class IntakeConstants{
        /**Speed that the intake will run at unless directly overridden by the runAtPercent command */
        public static final double DEFAULT_INTAKE_SPEED = 0.5;
    }
}