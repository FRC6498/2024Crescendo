// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends SubsystemBase {

  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
    .withDeadband(DriveConstants.maxSpeed.in(MetersPerSecond) * 0.1).withRotationalDeadband(DriveConstants.maxAngularRate.in(RadiansPerSecond) * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
  /** Creates a new Drivetrain. */
  public Drivetrain(Telemetry logger) {
    if (Utils.isSimulation()) {
      swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    swerve.registerTelemetry(logger::telemeterize);
  }

  public Command driveFieldRelative(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier omegaVelocity) {
    return run(() -> swerve.setControl(
      driveFieldCentric.withVelocityX(-xVelocity.getAsDouble() * DriveConstants.maxSpeed.in(MetersPerSecond))
        .withVelocityY(-yVelocity.getAsDouble() * DriveConstants.maxSpeed.in(MetersPerSecond))
        .withRotationalRate(-omegaVelocity.getAsDouble() * DriveConstants.maxAngularRate.in(RadiansPerSecond))
      )
    );
  }

  public Command brake() {
    return runOnce(() -> swerve.setControl(brake));
  }

  public Pose2d getRobotPose2d() {
    return swerve.getState().Pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
