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
  private double MaxSpeed = 10;
  private double MaxAngularRate = 8.6* Math.PI;
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController driverController, operatorController;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
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
 // private final Telemetry logger;
  private final Intake intakeSub;
  private final Climber climber;
  private final Shooter shooterSub;
  private final Arm armSub;
  private final Leds ledSub;
  public RobotContainer() {
   
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    //logger = new Telemetry(MaxSpeed);
    intakeSub = new Intake();
    climber = new Climber();
    shooterSub = new Shooter();
    ledSub = new Leds();
    armSub = new Arm();
    NamedCommands.registerCommand("ShootSpeakerCommand", ShootSpeakerClose());
    NamedCommands.registerCommand("IntakeCommand", intakeToArmAuto());
    NamedCommands.registerCommand("ShootSpeakerDistance", ShootSpeakerDistance());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    ledSub.setDefaultCommand(ledSub.StartLeds());
    configureBindings();
  }
  private void configureBindings() {
    //#region drive commands

    //* drive fieldcentric by default */
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FieldCentricDrive
          .withVelocityX(driverController.getLeftY() * MaxSpeed)
          .withVelocityY(driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        ));
    //* brake the drive motors */
    driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    //* go back to field centric drive */
    driverController.x().onTrue(
        drivetrain.applyRequest(
          () -> FieldCentricDrive
          .withVelocityX(-driverController.getLeftY() * MaxSpeed)
          .withVelocityY(-driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        )
      );
    driverController.y().onTrue(drivetrain.applyRequest(() -> FieldCentricDrive
          .withVelocityX(driverController.getLeftY() * MaxSpeed)
          .withVelocityY(driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        ));
//#endregion
      //operator bumpers run the climber
    operatorController.leftBumper().onTrue(climber.Run()).onFalse(climber.Stop());
    operatorController.rightBumper().onTrue(climber.Reverse()).onFalse(climber.Stop());
      //operator A reverses the arm intake
    operatorController.a().onTrue(ShootAmp());
     //operator B lowers the arm and runs both intakes
    operatorController.b().onTrue(intakeToArmAuto()).onFalse(intakeSub.stop().andThen(intakeSub.StopArmIntake()));
    //operator X runs the arm intake
    operatorController.x().onTrue(ShootSpeakerDistance());
      //operator Y runs the shooter wheels
    operatorController.y().onTrue(ShootSpeakerClose());
      //operator dpad up reverses the main intake
    operatorController.pov(0).onTrue(ReverseAllIntakes()).onFalse(intakeSub.StopIntakes());

    operatorController.pov(90).onTrue(PassShot());
    // operatorController.pov(90).onTrue(Commands.runOnce(()->{drivetrain.resetPose();}, drivetrain));
      //operator dpad down moves the arm down
    operatorController.pov(180).onTrue(resetArm());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    //drivetrain.registerTelemetry(logger::telemeterize);
  }
  private Command intakeToArmAuto() {
      return intakeSub.RunIntakes().until(()->intakeSub.intakeHasNote)
        .andThen(intakeSub.StopArmIntake())
        .andThen(intakeSub.stop());
        }
  public Command resetArm() {
    return Commands.runOnce(()-> {armSub.setGoal(0); armSub.enable();}, armSub).andThen(shooterSub.stop());
  }
  public Command ReverseAllIntakes() {
    return intakeSub.ReverseArmIntake().andThen(intakeSub.Reverse());
  }
  private Command ShootSpeaker() { 
    return
    intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(0.075 /* rotations */); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(130).until(()-> Math.abs(130 - shooterSub.GetShooterAverageRpm()) < 10) /* prob broken */)
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  private Command ShootSpeakerDistance() {
    return
    intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(Constants.ShooterConstants.CalcShooterAngleFromDistance(drivetrain.GetDistanceToSpeaker())); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(0.75))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
    private Command ShootSpeakerClose() {
    return intakeSub.IntakeMain()
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(.5))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
   private Command PassShot() {
    return
    Commands.runOnce(()->{armSub.setGoal(0.065); armSub.enable();}, armSub)
    .andThen(intakeSub.IntakeMain())
    .andThen(shooterSub.RunAtVelocity(70))
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  
  private Command ShootAmp() {
    return intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(0.22); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(40))
    .andThen(new WaitCommand(0.75))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(0.75))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
}
