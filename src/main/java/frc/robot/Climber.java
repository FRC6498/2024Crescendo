// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX climberMotor;
  public Climber() {
    climberMotor = new TalonFX(0);
  }
  public Command Run(double percent){
    return this.runOnce(()-> climberMotor.setControl(new VoltageOut(12*percent)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
