// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Shooter;
import frc.robot.Vision;

public class ShootWhileMoving extends Command {
  /** Creates a new ShootWhileMoving. */
  private final Shooter shooterSub;
  private final CommandSwerveDrivetrain drivetrainSub;
  private final Vision visionSub; 

  private double distanceToSpeaker;
  private double angleToSpeaker;
  private double speedPerpendicularToSpeaker;
  private double shotVelocityForCurrentDistance;

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
    distanceToSpeaker = visionSub.GetRobotToSpeakerDistance();
    speedPerpendicularToSpeaker = drivetrainSub.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
    
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
