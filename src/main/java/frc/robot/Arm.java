// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armMotor;
  private final CANcoder armEncoder;
  public Arm() {
    armMotor = new TalonFX(0);
    armEncoder = new CANcoder(0);
    TalonFXConfiguration config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withFeedbackRemoteSensorID(1));
    armMotor.getConfigurator().apply(config);
    armMotor.getConfigurator().apply(Constants.ArmConstants.armConfigs);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
