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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 10; // max allowable speed of the robot in m/s (very optimistic to uncap the speed in auto)
  private double MaxAngularRate = 8.6* Math.PI; // max allowable rotational speed of the robot in rad/s (dialed back to make the robot more controllable)
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController driverController, operatorController;
  // create the drivetrain from the Constants created when generating the swerve code in Tuner
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
//#region Swerve drive commands
  // drives the robot with input being relative to the field (from Phoenix swerve library)
  private final SwerveRequest.FieldCentric FieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1) // margin of translational error
      .withRotationalDeadband(MaxAngularRate * 0.1) // margin of rotational error
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // never learned what this did
  /*
    drives the robot with input being relative to the robot
    (forward on the stick is always forward on the robot)
    (from Phoenix swerve library)
  */
  private final SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // stop the robot from driving and set all the motors to brake
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // point the robot to a field relative angle (never got this to work)
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  /*
    planned to make a modified drive command with constant rotational input
    so the driver could spin while driving but never did it
  */
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
    // create controllers
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    //logger = new Telemetry(MaxSpeed); (disabled to try to fix memory issue on RIO 1. IDK if this actually did anything)
    intakeSub = new Intake();
    climber = new Climber();
    shooterSub = new Shooter();
    ledSub = new Leds();
    armSub = new Arm();
    // Register Commands so PathPlanner can use them
    NamedCommands.registerCommand("ShootSpeakerCommand", ShootSpeakerClose());
    NamedCommands.registerCommand("IntakeCommand", intakeToArmAuto());
    NamedCommands.registerCommand("ShootSpeakerDistance", ShootSpeakerDistance());
    autoChooser = AutoBuilder.buildAutoChooser();
    // put auto choice selecter on the Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // start the Leds
    ledSub.setDefaultCommand(ledSub.StartLeds());
    configureBindings();
  }
  private void configureBindings() {
    //#region drive commands
    //* drive fieldcentric by default */
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> FieldCentricDrive // drive Field-Centric
          .withVelocityX(driverController.getLeftY() * MaxSpeed) // pass driver controller Left stick Y values to command
          .withVelocityY(driverController.getLeftX() * MaxSpeed) // pass driver controller Left stick Y values to command
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // pass driver controller Left stick Y values to command
        ));
    //* brake the drive motors */
    driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    //* invert the driver's controls to fix issue with robot orientation (dirty fix because this didn't fix the actual problem) */
    driverController.x().onTrue(
        drivetrain.applyRequest(
          () -> FieldCentricDrive
          .withVelocityX(-driverController.getLeftY() * MaxSpeed)
          .withVelocityY(-driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        )
      );
    //* Uninvert the driver's controls */
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
      //operator dpad down moves the arm down
    operatorController.pov(180).onTrue(resetArm());

    // set the inital robot position  !! only used in simulation !!
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // diabled to try to fix memory issue on rio 1
    //drivetrain.registerTelemetry(logger::telemeterize);
  }
  /** run both intakes untill the arm sensor detects a note */
  private Command intakeToArmAuto() {
      return intakeSub.RunIntakes().until(intakeSub.armIntakeHasNote /* run untill the trigger detects a note */)
        .andThen(intakeSub.StopArmIntake())
        .andThen(intakeSub.stop());
        }
  /** moves the arm back to default position */
  public Command resetArm() {
    return Commands.runOnce(()-> { armSub.setGoal(0); armSub.enable(); }, armSub).andThen(shooterSub.stop());
  }
  public Command ReverseAllIntakes() {
    return intakeSub.ReverseArmIntake().andThen(intakeSub.Reverse());
  }
  /** Long distance speaker shot from fixed position  */
  private Command ShootSpeaker() {
    return
    intakeSub.IntakeMain().andThen( // run the main intake while we are shooting so the note does not get stuck in the mechanism
    Commands.runOnce(()->{armSub.setGoal(0.075 /* rotations */); armSub.enable();}, armSub)) // raise the arm
    .andThen(shooterSub.RunAtVelocity(130).until(()-> Math.abs(130 - shooterSub.GetShooterAverageRpm()) < 10 /* probably broken */) ) // run the shooter motors untill they reach 130 rpm average
    // do that again but with time because the rpm is broken (never tried to figure out why rpm didn't work)
    .andThen(shooterSub.RunAtVelocity(130))
    .andThen(new WaitCommand(1))
    // run the arm intake to shoot the note out of the mechanism
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(1))
    // stop everything and reset
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  /** Automatic distance speaker shot using the estimated pose of the robot */
  private Command ShootSpeakerDistance() {
    // Process same as above
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
  /** Shoot into the speaker while the robot is directly up against the speaker */
  private Command ShootSpeakerClose() {
    // Process the same as other shoot commands but with less wait time
    return intakeSub.IntakeMain()
    .andThen(shooterSub.RunAtVelocity(130))
    // arm does not need to be moved because bot is against the speaker
    .andThen(new WaitCommand(0.5))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(.5))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }
  /** Pass a note across the field */
  private Command PassShot() {
    // works the same as a shoot command but the arm goes up more
    return
    Commands.runOnce(()->{armSub.setGoal(0.065/* rotations */); armSub.enable();}, armSub)
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
  /** Shoots a note into the amp */
  private Command ShootAmp() {
    // works the same as a shoot command but with lower speed and max arm angle
    return intakeSub.IntakeMain().andThen(
    Commands.runOnce(()->{armSub.setGoal(0.22 /* rotations */); armSub.enable();}, armSub))
    .andThen(shooterSub.RunAtVelocity(40))
    .andThen(new WaitCommand(1))
    .andThen(intakeSub.IntakeArm())
    .andThen(new WaitCommand(0.75))
    .andThen(intakeSub.StopArmIntake())
    .andThen(intakeSub.stop())
    .andThen(shooterSub.stop())
    .andThen(Commands.runOnce(()->{armSub.setGoal(0); armSub.enable();}, armSub));
  }

  public Command getAutonomousCommand() {
   return autoChooser.getSelected(); // use the auto selected by the drivers on the dashboard
  }
}
