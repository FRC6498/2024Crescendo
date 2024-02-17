// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.spec.ECFieldF2m;
import java.util.function.BooleanSupplier;

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
  private double MaxSpeed = 6;
  private double MaxAngularRate = 1.5 * Math.PI;
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController driverController, operatorController;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
//#region Swerve drive commands
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
//#endregion
  private final Telemetry logger;
  private final Intake intakeSub;
  private final Climber climber;
  private final Shooter shooterSub;
  private final Arm armSub;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    logger = new Telemetry(MaxSpeed);
    intakeSub = new Intake();
    climber = new Climber();
    shooterSub = new Shooter();
    armSub = new Arm();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }
  private void configureBindings() {
    //#region drive commands
    //* drive fieldcentric by default */
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FieldCentricDrive
          .withVelocityX(-driverController.getLeftY() * MaxSpeed)
          .withVelocityY(-driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        ));
    //* brake the drive motors */
    driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    //* go back to field centric drive */
    driverController.leftBumper().whileTrue(drivetrain.run(()->
      drivetrain.applyRequest(
          () -> FieldCentricDrive
          .withVelocityX(-driverController.getLeftY() * MaxSpeed)
          .withVelocityY(-driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
          )
        )
      );
    driverController.x().onTrue(drivetrain.run(()-> drivetrain.applyRequest(
      ()-> RobotCentricDrive
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate))));
    driverController.y().onTrue(drivetrain.run(
      ()-> faceAngle
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(drivetrain.getRobotToSpeakerRotation())
        ));
//#endregion
    operatorController.leftBumper().onTrue(climber.Run()).onFalse(climber.Stop());
    operatorController.rightBumper().onTrue(climber.Reverse()).onFalse(climber.Stop());
    operatorController.a().onTrue(intakeSub.Run()).onFalse(intakeSub.stop());
    operatorController.b().onTrue(armSub.RotateToAbsoluteAngle(new Rotation2d(0.17))); // rotates arm to intake position
    operatorController.x().onTrue(armSub.RotateToAbsoluteAngle(new Rotation2d(1.571))); // rotates arm to amp position

    armSub.setDefaultCommand(intakeToArm(intakeSub.GetSensor(), armSub.getArmAtBottom()));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  private Command intakeToArm(Boolean intakeHasNote, Boolean armAtBottom) {
    return armSub.runOnce(()-> armSub.IntakeArm()).onlyIf(()-> intakeHasNote && armAtBottom);
  }
  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
