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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric FieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //private final Intake intakeSub = new Intake();
  private final Climber climber = new Climber();
  //private final Shooter shooterSub = new Shooter();

  private void configureBindings() {
    //* drive fieldcentric by default */
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FieldCentricDrive
          .withVelocityX(-driveController.getLeftY() * MaxSpeed)
          .withVelocityY(-driveController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driveController.getRightX() * MaxAngularRate)
        ));
    //* brake the drive motors */
    driveController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    //* go back to field centric drive */
    driveController.leftBumper().whileTrue(drivetrain.run(()->
      drivetrain.applyRequest(
          () -> FieldCentricDrive
          .withVelocityX(-driveController.getLeftY() * MaxSpeed)
          .withVelocityY(-driveController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driveController.getRightX() * MaxAngularRate)
          )
        )
      );
    //driveController.a().onTrue(ShootSpeaker());
    // driveController.a().onTrue(shooterSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driveController.b().onTrue(shooterSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driveController.a().onTrue(climber.Run()).onFalse(climber.Stop());
    driveController.b().onTrue(climber.Reverse()).onFalse(climber.Stop());
    //* run drive in RobotCentric
    driveController.x().onTrue(drivetrain.run(()-> drivetrain.applyRequest(
      ()-> RobotCentricDrive
        .withVelocityX(-driveController.getLeftY() * MaxSpeed)
        .withVelocityY(-driveController.getLeftX() * MaxSpeed)
        .withRotationalRate(-driveController.getRightX() * MaxAngularRate))));
    //* face the speaker depending on what alliance you are on */
    driveController.y().onTrue(drivetrain.run(
      ()-> faceAngle
        .withVelocityX(-driveController.getLeftY() * MaxSpeed)
        .withVelocityY(-driveController.getLeftX() * MaxSpeed)
        .withTargetDirection(drivetrain.getRobotToSpeakerRotation())
        ));
    // driveController.x().whileTrue(intakeSub.Run()).onFalse(intakeSub.stop());
    // driveController.y().whileTrue(intakeSub.Reverse()).whileFalse(intakeSub.stop());
    //driveController.y().whileTrue(shooterSub.RunAtVelocity(1));
    // driveController.pov(0).whileTrue(shooterSub.Run()).whileFalse(shooterSub.stop());
    // reset the field-centric heading on left bumper press
    //driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // driveController.leftBumper().onTrue(shooterSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driveController.rightBumper().onTrue(shooterSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  // public Command ShootSpeaker(){
  //   return
  //   intakeSub.Reverse()
  //   .andThen(shooterSub.Run())
  //   .andThen(new WaitCommand(0.3))
  //   .andThen(intakeSub.stop())
  //   .andThen(new WaitCommand(0.5))
  //   .andThen(intakeSub.Run())
  //   .andThen(new WaitCommand(0.5))
  //   .andThen(shooterSub.stop())
  //   .andThen(intakeSub.stop());
  // }

  public RobotContainer() {
   // NamedCommands.registerCommand("IntakeCommand", intakeSub.Run().andThen(new WaitCommand(1)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
