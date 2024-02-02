// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
  DigitalInput climberSensor;
  private final TalonFX climberMotor;
  Trigger climberAtMax;
  
  public Climber() {
    climberMotor = new TalonFX(16);
    climberSensor = new DigitalInput(1);
    climberAtMax = new Trigger(()->!climberSensor.get());
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public Command Run(){
    return this.run(()->climberMotor.set(-1)).until(climberAtMax).andThen(()->climberMotor.set(0));
    //return this.runOnce(()-> climberMotor.set(-0.2));
  }
  public Command Reverse() {
    return this.runOnce(()-> climberMotor.set(1));
  }
  public Command Stop() {
    return this.runOnce(()-> climberMotor.set(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climber Sensor", climberAtMax.getAsBoolean());
    // This method will be called once per scheduler run
  }
}
