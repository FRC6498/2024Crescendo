package frc.robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  // shooter motors
  private CANSparkMax topMotor, bottomMotor;
  // pid controllers for each of the shooter motors
  private SparkPIDController topPID, bottomPID;
  // voltage applied to the motors (for sysid)
  private final MutableMeasure<Voltage> appliedTopVoltage, appliedBottomVoltage;
  // angle of the motors (position)(for sysid)
  private final MutableMeasure<Angle> topDistance, bottomDistance;
  // velocity of the motors (for sysid)
  private final MutableMeasure<Velocity<Angle>> topVelocity, bottomVelocity;
  // create a new sysid routine
  public final SysIdRoutine routine;
  // feedforward for the shooter motors
  SimpleMotorFeedforward topFF, bottomFF;

  public Shooter() {
    topMotor = new CANSparkMax(16, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(17, MotorType.kBrushless);
    topMotor.setInverted(false);
    bottomMotor.setInverted(false);
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    // make the motors generate a pid controller for themselves
    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();

    // Configure motor pid and feedforward
    bottomPID.setP(TOP_MOTOR_KP);
    bottomPID.setI(TOP_MOTOR_KI);
    bottomPID.setD(TOP_MOTOR_KD);
    bottomPID.setFF(0.00015);
    bottomFF = new SimpleMotorFeedforward(TOP_MOTOR_KS, TOP_MOTOR_KV, TOP_MOTOR_KA);
    topPID.setP(BOTTOM_MOTOR_KP);
    topPID.setI(BOTTOM_MOTOR_KI);
    topPID.setD(BOTTOM_MOTOR_KD);
    bottomPID.setFF(0.00015);
    topFF = new SimpleMotorFeedforward(BOTTOM_MOTOR_KS, BOTTOM_MOTOR_KV, BOTTOM_MOTOR_KA);
//#region sysid setup
    //read the wpilib docs on sysid for explanation
    appliedTopVoltage = mutable(Volts.of(0));
    topDistance = mutable(Rotations.of(0));
    topVelocity = mutable(RPM.of(0));
    appliedBottomVoltage = mutable(Volts.of(0));
    bottomDistance = mutable(Rotations.of(0));
    bottomVelocity = mutable(RPM.of(0));
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              topMotor.setVoltage(volts.in(Volts));
              bottomMotor.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("top-motor")
                  .voltage(appliedTopVoltage
                      .mut_replace(topMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(topDistance.mut_replace(topMotor.getEncoder().getPosition() / 48, Rotations))
                  .angularVelocity(topVelocity.mut_replace(topMotor.getEncoder().getVelocity(), RPM));
              log.motor("bottom-motor")
                  .voltage(appliedBottomVoltage
                      .mut_replace(bottomMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(bottomDistance.mut_replace(bottomMotor.getEncoder().getPosition() / 48, Rotations))
                  .angularVelocity(bottomVelocity.mut_replace(bottomMotor.getEncoder().getVelocity(), RPM));
            }, this));
//#endregion
  }
  /** Runs both shooter motors at different speeds */
  public Command RunAtVelocity(double topMotorVelocity, double bottomMotorVelocity) {
        return this.runOnce(() -> topPID.setReference(topMotorVelocity, ControlType.kVelocity, 0, topFF.calculate(topMotorVelocity)))
        .andThen(
            this.run(
                () -> bottomPID.setReference(bottomMotorVelocity, ControlType.kVelocity, 0, bottomFF.calculate(bottomMotorVelocity))));
  }
  /** Runs both shooter motors at the same speed */
  public Command RunAtVelocity(double velocity) {
    return this.runOnce(() -> topPID.setReference(velocity, ControlType.kVelocity, 0, topFF.calculate(velocity)))
        .andThen(
            this.runOnce(
                () -> bottomPID.setReference(velocity, ControlType.kVelocity, 0, bottomFF.calculate(velocity))));
  }
  /** runs both shooter motors off of precent output */
  public Command Run() {
    return this.runOnce(() -> topMotor.set(DEFAULT_SHOOTER_SPEED)).andThen(() -> bottomMotor.set(DEFAULT_SHOOTER_SPEED));
  }
  /** stops both shooter motors */
  public Command stop() {
    return this.runOnce(() -> topMotor.set(0)).andThen(() -> bottomMotor.set(0));
  }
  public double GetShooterAverageRpm() {
    return (topMotor.getEncoder().getVelocity() + bottomMotor.getEncoder().getVelocity()) / 2; // average the encoder velocitys
  }
  /** runs the first sysid test */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //reset the encoders to improve data
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
    // run the test
    return routine.quasistatic(direction);
  }
  /** runs the second sysid test */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // reset the enocders to improve data
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
    // run the test
    return routine.dynamic(direction);
  }
  @Override
  public void periodic() {
  }
}
