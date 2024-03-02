package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX MainIntakeMotor;
  CANSparkMax ArmIntakeMotor;
  DigitalInput ArmIntakeSensor;
  public Trigger mainIntakeHasNote, armIntakeHasNote;
  public Intake() {
    MainIntakeMotor = new TalonFX(10);
    ArmIntakeMotor = new CANSparkMax(18, MotorType.kBrushless);
    ArmIntakeSensor = new DigitalInput(1);
    MainIntakeMotor.setInverted(false);
    ArmIntakeMotor.setInverted(true);
    armIntakeHasNote = new Trigger(()-> !ArmIntakeSensor.get());
  }
  public Command IntakeMain() {
    return this.runOnce(()-> MainIntakeMotor.set(0.5));
  }
  public Command IntakeArm() {
    return this.runOnce(()-> ArmIntakeMotor.set(0.3));
  }
  public Command StopArmIntake() {
    return this.runOnce(()-> ArmIntakeMotor.set(0));
  }
  public Command runAtPrecent(double percent) { return this.runOnce(()-> MainIntakeMotor.set(percent)); }
  public Command Reverse() { return runAtPrecent(-IntakeConstants.DEFAULT_INTAKE_SPEED); }
  public Command ReverseArmIntake() {return this.runOnce(()-> ArmIntakeMotor.set(-0.2));}
  public Command stop() { return runAtPrecent(0); }
  public Boolean getIntakeSensor() {
    return !ArmIntakeSensor.get();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake sensor", !ArmIntakeSensor.get());
    
  }
}
