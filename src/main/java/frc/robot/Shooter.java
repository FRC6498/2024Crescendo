// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax topMotor, bottomMotor;
  public Shooter() {
    topMotor = new CANSparkMax(14, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(15, MotorType.kBrushless);
    Slot0Configs configs = new Slot0Configs();
    
  }
  public Command RunAtVelocity(double topMotorVelocity, double bottomMotorVelocity){
    return this.runOnce(()->
      {
        //topMotor.setControl(new VelocityDutyCycle(topMotorVelocity));
        //bottomMotor.setControl(new VelocityDutyCycle(bottomMotorVelocity));
      }
      );
  }
  public Command RunAtVelocity(double velocity){
    return this.runOnce(()->{
       // topMotor.setControl(new VelocityDutyCycle(velocity));
       // bottomMotor.setControl(new VelocityDutyCycle(velocity));
    });
  }
  public Command RunAtPercent(double percent){
    return this.runOnce(()->
      topMotor.set(percent)).andThen(()->bottomMotor.set(percent)
    );
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
