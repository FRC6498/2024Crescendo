// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax topMotor, bottomMotor;
  private SparkPIDController topPID, bottomPID;
  //private double topMotorCurrentVelocity, bottomMotorCurrentVelocity;
  double ShooterSpeedPercent = 1;
  private final MutableMeasure<Voltage> appliedTopVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));
  public final SysIdRoutine routine;
  SimpleMotorFeedforward topFF, bottomFF;

  public Shooter() {
    topMotor = new CANSparkMax(14, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(15, MotorType.kBrushless);

    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPositionConversionFactor(0);

    topMotor.setInverted(true);
    bottomMotor.setInverted(true);

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();

    topPID.setP(0.011415);
    topPID.setI(0.0);
    topPID.setD(0.0);
    topFF = new SimpleMotorFeedforward(-0.1924, 0.13174, 0.07513);

    bottomPID.setP(0.007482);
    bottomPID.setI(0.0);
    bottomPID.setD(0.0);
    bottomFF = new SimpleMotorFeedforward(0.12187, 0.12947, 0.032151);

    // sysid
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
                  .angularPosition(distance.mut_replace(topMotor.getEncoder().getPosition() / 48, Rotations))
                  .angularVelocity(velocity.mut_replace(topMotor.getEncoder().getVelocity(), RPM));
              log.motor("bottom-motor")
                  .voltage(appliedTopVoltage
                      .mut_replace(bottomMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(distance.mut_replace(bottomMotor.getEncoder().getPosition() / 48, Rotations))
                  .angularVelocity(velocity.mut_replace(bottomMotor.getEncoder().getVelocity(), RPM));
            }, this));

  }

  /**
   * Runs the shooter motors at different velocitys
   * 
   * @param topMotorVelocity
   * @param bottomMotorVelocity
   * @return
   */
  public Command RunAtVelocity(double topMotorVelocity, double bottomMotorVelocity) {
    return this.runOnce(() -> topPID.setReference(topMotorVelocity, ControlType.kVelocity))
        .alongWith(
            this.runOnce(() -> bottomPID.setReference(bottomMotorVelocity, ControlType.kVelocity)));
  }

  /**
   * Runs both shooter motors at the same velocity
   * 
   * @param velocity
   *                 Velocity to run both motors at (rpm?)
   * @return
   */
  public Command RunAtVelocity(double velocity) {
    return this.runOnce(() -> topPID.setReference(velocity, ControlType.kVelocity, 0, topFF.calculate(velocity)))
        .andThen(
            this.runOnce(
                () -> bottomPID.setReference(velocity, ControlType.kVelocity, 0, bottomFF.calculate(velocity))));
  }

  public Command Run() {
    return this.runOnce(() -> topMotor.set(ShooterSpeedPercent)).andThen(() -> bottomMotor.set(ShooterSpeedPercent));
  }

  public Command stop() {
    return this.runOnce(() -> topMotor.set(0)).andThen(() -> bottomMotor.set(0));
  }

  public double GetShooterAverageRpm() {
    return (topMotor.getEncoder().getVelocity() + bottomMotor.getEncoder().getVelocity()) / 2;
  }

  public double GetApproxExitVelocity() {
    // TODO: figure out rpm vs exit velo function for the shooter
    return 0;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // topMotorCurrentVelocity = topMotor.getEncoder().getVelocity();
    // bottomMotorCurrentVelocity = topMotor.getEncoder().getVelocity();
  }
}
