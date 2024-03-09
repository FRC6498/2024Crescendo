package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    CameraServer.startAutomaticCapture();
    NamedCommands.registerCommand("ShootSpeakerCommand", ShootSpeakerClose());
    NamedCommands.registerCommand("IntakeCommand", intakeToArmAuto());
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
      //operator bumpers run the climber
    operatorController.leftBumper().onTrue(climber.Run()).onFalse(climber.Stop());
    operatorController.rightBumper().onTrue(climber.Reverse()).onFalse(climber.Stop());
      //operator A reverses the arm intake
    operatorController.a().onTrue(intakeSub.ReverseArmIntake()).onFalse(intakeSub.StopArmIntake());
      //operator Y runs the shooter wheels
    operatorController.y().onTrue(ShootSpeaker());
      //operator B lowers the arm and runs both intakes
    operatorController.b().onTrue(intakeToArmAuto()).onFalse(intakeSub.stop().andThen(intakeSub.StopArmIntake()));
      //operator X runs the arm intake
    operatorController.x().onTrue(ShootSpeakerClose());
      //operator dpad up reverses the main intake
    operatorController.pov(0).onTrue(intakeSub.ReverseArmIntake()).onFalse(intakeSub.StopArmIntake());
      //operator xpad right moves the arm up
    operatorController.pov(90).onTrue(ShootAmp());
      //operator dpad down moves the arm down
    operatorController.pov(180).onTrue(Commands.runOnce(()->{armSub.setGoal(0);}, armSub));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  private Command intakeToArmAuto() {
      return intakeSub.RunIntakes().until(()->intakeSub.intakeHasNote)
        .andThen(intakeSub.StopArmIntake())
        .andThen(intakeSub.stop());
  }
  private Command ShootSpeaker() {
    return
    intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(0.07); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(130).until(()-> Math.abs(130 - shooterSub.GetShooterAverageRpm()) < 10))
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  private Command ShootSpeakerDistance(double rotations) {
    return
    intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(rotations); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(130).until(()-> Math.abs(130 - shooterSub.GetShooterAverageRpm()) < 10))
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
    private Command ShootSpeakerClose() {
    return
    intakeSub.IntakeMain()
    .andThen(shooterSub.RunAtVelocity(130).until(()-> Math.abs(130 - shooterSub.GetShooterAverageRpm()) < 10))
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(1.5))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1.5))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop());
  }
  
  private Command ShootAmp() {
    return intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(0.25); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(40).until(()-> Math.abs(10 - shooterSub.GetShooterAverageRpm()) < 10))
    .andThen(shooterSub.RunAtVelocity(40))
    .andThen(new WaitCommand(1.2))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
