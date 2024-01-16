// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final Intake intakeSub = new Intake();
  private final Shooter shooterSub = new Shooter();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driveController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController.leftBumper().whileTrue(
      drivetrain.applyRequest(
        () -> point.withModuleDirection(
          new Rotation2d(
            -driveController.getLeftY(),
            -driveController.getLeftX()
          )
        )
      )
     );
    driveController.a().onTrue(ShootSpeaker());
    driveController.x().whileTrue(intakeSub.Reverse()).whileFalse(intakeSub.stop());
    driveController.y().whileTrue(shooterSub.RunAtVelocity(1));
    driveController.pov(0).whileTrue(shooterSub.RunAtPercent(-.50)).whileFalse(shooterSub.RunAtPercent(0));
    // reset the field-centric heading on left bumper press
    driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  public Command ShootSpeaker(){
    return 
    intakeSub.Reverse()
    .andThen(shooterSub.RunAtPercent(-0.5))
    .andThen(new WaitCommand(0.3))
    .andThen(intakeSub.stop())
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.Run())
    .andThen(new WaitCommand(0.5))
    .andThen(shooterSub.stop())
    .andThen(intakeSub.Run());
  }

  public RobotContainer() {
    NamedCommands.registerCommand("IntakeCommand", intakeSub.Run().andThen(new WaitCommand(1)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
