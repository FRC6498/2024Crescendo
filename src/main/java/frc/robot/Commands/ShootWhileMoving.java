// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Shooter;
import frc.robot.Vision;

public class ShootWhileMoving extends Command {
  /** Creates a new ShootWhileMoving. */
  private final Shooter shooterSub;
  private final CommandSwerveDrivetrain drivetrainSub;
  private final Vision visionSub; 

  //variables gathered from data on the robot
  private double distanceToSpeaker;
  private double angleToSpeaker;
  private double speedPerpendicularToSpeaker;
  private double shotVelocityForCurrentDistance;
  private Rotation2d currentRotation;
  
  //calculated variables
  private double flightTime;
  private double goalDeltaX;

  //final variables
  private Rotation2d CorrectedShotAngle;
  private double NewShotDistance;

  public ShootWhileMoving(Shooter shooterSub, CommandSwerveDrivetrain drivetrainSub, Vision visionSub) {
    addRequirements(shooterSub, drivetrainSub, visionSub);
    this.shooterSub = shooterSub;
    this.visionSub = visionSub;
    this.drivetrainSub = drivetrainSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedPerpendicularToSpeaker = drivetrainSub.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
    currentRotation =  drivetrainSub.getRotation3d().toRotation2d();

    //these make a right triangle
    distanceToSpeaker = visionSub.GetRobotToSpeakerDistance();
    goalDeltaX = (shooterSub.GetApproxExitVelocity()*distanceToSpeaker) * speedPerpendicularToSpeaker;

    //trig to find angle after robot has moved    
    CorrectedShotAngle = currentRotation.plus(new Rotation2d(Math.atan(goalDeltaX/distanceToSpeaker)));

    //pythagorean therom to find the new shot distance
    NewShotDistance = Math.sqrt(Math.pow(distanceToSpeaker, 2) + Math.pow(goalDeltaX, 2));

    //correct for shot

    //if we are within 5 degrees then stop trying to correct
    if(Math.abs(currentRotation.getDegrees() - CorrectedShotAngle.getDegrees()) > 5){
      drivetrainSub.applyRequest(()-> new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(CorrectedShotAngle));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
