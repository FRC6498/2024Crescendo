// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX topMotor, bottomMotor;
  public Shooter() {
    topMotor = new TalonFX(15);
    bottomMotor = new TalonFX(16);
    Slot0Configs configs = new Slot0Configs();
    configs.kP = 0;
    configs.kI = 0;
    configs.kD = 0;
    topMotor.getConfigurator().apply(configs);
    bottomMotor.getConfigurator().apply(configs);
  }
  public Command RunAtVelocity(double topMotorVelocity, double bottomMotorVelocity){
    return this.runOnce(()->
      {
        topMotor.setControl(new VelocityDutyCycle(topMotorVelocity));
        bottomMotor.setControl(new VelocityDutyCycle(bottomMotorVelocity));
      }
      );
  }
  public Command RunAtVelocity(double velocity){
    return this.runOnce(()->{
        topMotor.setControl(new VelocityDutyCycle(velocity));
        bottomMotor.setControl(new VelocityDutyCycle(velocity));
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
