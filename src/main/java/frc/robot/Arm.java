// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final TalonFX LeftArmMotor, RightArmMotor, ArmIntake;
  private final ArmFeedforward armFeedforward;
  private final SysIdRoutine armIdRoutine;
  private final MutableMeasure<Voltage> RightAppliedVoltage, LeftAppliedVoltage;
  private final MutableMeasure<Angle> RightAngle, LeftAngle;
  private final MutableMeasure<Velocity<Angle>> RightVelocity, LeftVelocity;
  public Arm() {
    LeftArmMotor = new TalonFX(19);
    RightArmMotor = new TalonFX(15);

    LeftArmMotor.setPosition(0);
    RightArmMotor.setPosition(0);

    ArmIntake = new TalonFX(18);
    Slot0Configs configs = ArmConstants.armConfigs;
    LeftArmMotor.getConfigurator().apply(configs);
    armFeedforward = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KG, ArmConstants.ARM_KV);
    RightArmMotor.getConfigurator().apply(configs);

//#region sysid setup
    RightAppliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    LeftAppliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    RightAngle = MutableMeasure.mutable(Units.Rotations.of(0));
    LeftAngle = MutableMeasure.mutable(Units.Rotations.of(0));
    RightVelocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));
    LeftVelocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));
    armIdRoutine= new SysIdRoutine(new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        LeftArmMotor.setVoltage(volts.in(Units.Volts));
        RightArmMotor.setVoltage(volts.in(Units.Volts));
      },
       log -> {
        log.motor("RightArmMotor")
          .voltage(RightAppliedVoltage.mut_replace(
            RightArmMotor.getMotorVoltage().getValue(), Units.Volts))
          .angularPosition(RightAngle.mut_replace(
            RightArmMotor.getPosition().getValue(), Units.Rotations))
          .angularVelocity(RightVelocity.mut_replace(
            RightArmMotor.getVelocity().getValue(), Units.RotationsPerSecond));
        log.motor("LeftArmMotor")
          .voltage(LeftAppliedVoltage.mut_replace(
            LeftArmMotor.getMotorVoltage().getValue(), Units.Volts))
          .angularPosition(LeftAngle.mut_replace(
            LeftArmMotor.getPosition().getValue(), Units.Rotations))
          .angularVelocity(LeftVelocity.mut_replace(
            LeftArmMotor.getVelocity().getValue(), Units.RotationsPerSecond));
       }, this));
//#endregion

  }
  public Command RotateToAbsoluteAngle(Rotation2d rotation){
    return this.runOnce(
      ()-> LeftArmMotor.setControl(
        new PositionDutyCycle(rotation.getRadians())
        .withFeedForward(
          armFeedforward.calculate(LeftArmMotor.getPosition().getValue(), LeftArmMotor.getVelocity().getValue())
          )
      )
    ).andThen(()-> RightArmMotor.setControl(
        new PositionDutyCycle(rotation.getRadians())
        .withFeedForward(
          armFeedforward.calculate(LeftArmMotor.getPosition().getValue(), LeftArmMotor.getVelocity().getValue())
          )
      ));
  }
  public Command RotateToRelativeAngle(Rotation2d rotation) {
    return this.runOnce(
      ()-> LeftArmMotor.setControl(
        new PositionDutyCycle(RightArmMotor.getPosition().getValue() + rotation.getRadians())
        .withFeedForward(
          armFeedforward.calculate(LeftArmMotor.getPosition().getValue(), LeftArmMotor.getVelocity().getValue())
          )
      )
    ).andThen(()-> RightArmMotor.setControl(
        new PositionDutyCycle(RightArmMotor.getPosition().getValue() + rotation.getRadians())
        .withFeedForward(
          armFeedforward.calculate(LeftArmMotor.getPosition().getValue(), LeftArmMotor.getVelocity().getValue())
          )
    ));
  }
  public Command IntakeArm() {
    return this.runOnce(()-> ArmIntake.set(0.5));
  }
  public Boolean getArmAtBottom() {
    if (LeftArmMotor.getPosition().getValue() < 0.2 ) {
      return true;
    }else{
      return false;
    }
  }

//#region sysid commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return armIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return armIdRoutine.dynamic(direction);
  }
//#endregion

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
