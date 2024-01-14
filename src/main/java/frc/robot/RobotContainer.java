// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  
  
  private final Telemetry logger = new Telemetry();
  private final Intake intakeSub = new Intake();
  private final Shooter shooterSub = new Shooter();
  private final Drivetrain driveSub = new Drivetrain(logger);

  private void configureBindings() {
    driveSub.setDefaultCommand(driveSub.driveFieldRelative(driveController::getLeftY, driveController::getLeftX, driveController::getRightX));
    driveController.a().whileTrue(driveSub.brake());
    driveController.x().whileTrue(intakeSub.run(-.75)).whileFalse(intakeSub.run(0));
    driveController.y().whileTrue(shooterSub.RunAtVelocity(1));
    driveController.pov(0).whileTrue(shooterSub.RunAtPercent(-.50)).whileFalse(shooterSub.RunAtPercent(0));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("ShootSpeakerCommand", shooterSub.ShootSpeaker());
    NamedCommands.registerCommand("IntakeCommand", intakeSub.run(50).andThen(new WaitCommand(1)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
