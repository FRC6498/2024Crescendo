// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax topMotor, bottomMotor;
  private final SparkPIDController topPID, bottomPID;
  private double topMotorCurrentVelocity, bottomMotorCurrentVelocity;
  
  public Shooter() {
    topMotor = new CANSparkMax(14, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(15, MotorType.kBrushless);

    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();

    topMotor.setInverted(true);
    bottomMotor.setInverted(true);    
  }
  /**
   * Runs the shooter motors at different velocitys
   * @param topMotorVelocity
   * @param bottomMotorVelocity
   * @return
   */
  public Command RunAtVelocity(double topMotorVelocity, double bottomMotorVelocity){
    return 
    this.runOnce(()-> topPID.setReference(topMotorVelocity, ControlType.kVelocity))
    .alongWith(
    this.runOnce(()->bottomPID.setReference(bottomMotorVelocity, ControlType.kVelocity))
    );
  }
  /**
   * Runs both shooter motors at the same velocity
   * @param velocity
   * Velocity to run both motors at (rpm?)
   * @return
   */
  public Command RunAtVelocity(double velocity){
    return
    this.runOnce(()-> topPID.setReference(velocity, ControlType.kVelocity))
    .alongWith(
    this.runOnce(()->bottomPID.setReference(velocity, ControlType.kVelocity))
    );
  }
  public Command RunAtPercent(double percent){
    return this.runOnce(()->
      topMotor.set(percent)).andThen(()->bottomMotor.set(percent)
    );
  }
  public Command stop() {
    return RunAtPercent(0);
  }
  

  @Override
  public void periodic() {
    topMotorCurrentVelocity = topMotor.getEncoder().getVelocity();
    bottomMotorCurrentVelocity = topMotor.getEncoder().getVelocity();
  }
}
