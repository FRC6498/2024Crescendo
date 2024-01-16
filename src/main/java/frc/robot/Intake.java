// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  public Intake() {
    intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(true);
  }
  public Command runAtPrecent(double percent) {
    return this.runOnce(()->intakeMotor.set(percent));
  }
  /**
   * Runs the intake at default speed in the forward direction
   * @return
   */
  public Command Run() {
    return runAtPrecent(IntakeConstants.DEFAULT_INTAKE_SPEED);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
