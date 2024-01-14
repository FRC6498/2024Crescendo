// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax topMotor, bottomMotor;
  public Shooter() {
    topMotor = new CANSparkMax(14, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(15, MotorType.kBrushless);    
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
  public Command ShootSpeaker() {
    return RunAtPercent(50).andThen(new WaitCommand(1)).andThen(stop());
  }
  public Command stop() {
    return this.runOnce(()->RunAtPercent(0));
  }
  public Command waitCommand(double seconds) {
    return new WaitCommand(seconds);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
