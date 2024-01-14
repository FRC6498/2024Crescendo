// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

/** Add your docs here. */
public final class Constants {
    public static final class VisionConstants{
      public static final Transform3d ROBOT_TO_CAMERA = new Transform3d();
    }

    public static final class FieldConstants{
      public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static final class DriveConstants {
      public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(6);
      public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(0.75);
    }
}
