package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;
  DigitalInput intakeSensor;
  boolean hasPiece;
  public Intake() {
    intakeMotor = new TalonFX(0);//TODO: get intake motor id
    intakeSensor = new DigitalInput(1);
    intakeMotor.setInverted(true);
  }
  public Command runAtPrecent(double percent) { return this.runOnce(()-> intakeMotor.set(percent)); }
  public Command Run() {
    if (hasPiece) {
      return this.runOnce(()->intakeMotor.set(0));
    }else{
      return this.runOnce(()->intakeMotor.set(IntakeConstants.DEFAULT_INTAKE_SPEED));
    }
  }
  public Command Reverse() { return runAtPrecent(-IntakeConstants.DEFAULT_INTAKE_SPEED); }
  public Command stop() { return runAtPrecent(0); }
  public boolean GetSensor() { return intakeSensor.get(); }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("inatake sensor state", intakeSensor.get());
  }
}
