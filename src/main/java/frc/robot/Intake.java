// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  AnalogTrigger intakeSensor;
  DigitalInput newSensor;
  boolean hasPiece;

  public Intake() {
    newSensor = new DigitalInput(1);
    intakeSensor = new AnalogTrigger(0);
    intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Intake Speed Percent", IntakeConstants.DEFAULT_INTAKE_SPEED);
    intakeSensor.setLimitsVoltage(0.5, 1.3);
  }
  public Command runAtPrecent(double percent) {
    return this.runOnce(()->intakeMotor.set(percent));
  }
  /**
   * Runs the intake at default speed in the forward direction
   * @return
   */
  public Command Run() {
    if (hasPiece) {
     return this.runOnce(()->intakeMotor.set(0)); 
    }else{
      return this.runOnce(()->intakeMotor.set(IntakeConstants.DEFAULT_INTAKE_SPEED));
    }
  }
  /**
   * Runs the intake at default speed in the reverse direction
   * @return
   */
  public Command Reverse() {
    return runAtPrecent(-IntakeConstants.DEFAULT_INTAKE_SPEED);
  }
  /**
   * Stops the intake
   * @return
   */
  public Command stop() {
    return runAtPrecent(0);
  }
  public boolean GetSensor() {
    return intakeSensor.getTriggerState();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake has piece", intakeSensor.getTriggerState());
    SmartDashboard.putBoolean("newIntakeSensor", newSensor.get());
    hasPiece = newSensor.get();
    // This method will be called once per scheduler run
  }
}
