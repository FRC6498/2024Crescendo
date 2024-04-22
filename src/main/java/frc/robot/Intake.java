package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  TalonFX MainIntakeMotor;
  CANSparkMax ArmIntakeMotor;
  /** Sensor on the arm that detects when the robot has a note */
  DigitalInput ArmIntakeSensor;
  /** Trigger that reads the value of the Intake sensor and flips when the senor detects a note */
  public Trigger armIntakeHasNote;
  public Intake() {
    MainIntakeMotor = new TalonFX(55);
    ArmIntakeMotor = new CANSparkMax(18, MotorType.kBrushless);
    ArmIntakeSensor = new DigitalInput(1);
    MainIntakeMotor.setInverted(false);
    ArmIntakeMotor.setInverted(true);
    ArmIntakeMotor.setIdleMode(IdleMode.kBrake);
    armIntakeHasNote = new Trigger(()-> !ArmIntakeSensor.get());
  }
  public Command IntakeMain() {
    return this.runOnce(()-> MainIntakeMotor.set(0.5));
  }
  public Command RunIntakes() {
    return this.run(()-> {ArmIntakeMotor.set(0.6); MainIntakeMotor.set(0.6);});
  }
  public Command StopIntakes() {
        return this.runOnce(()-> {ArmIntakeMotor.set(0); MainIntakeMotor.set(0);});

  }
  public Command IntakeArm() {
    return this.run(()-> ArmIntakeMotor.set(0.7)).until(armIntakeHasNote);
  }
  public Command StopArmIntake() {
    return this.runOnce(()-> ArmIntakeMotor.set(0));
  }
  public Command runAtPrecent(double percent) { return this.runOnce(()-> MainIntakeMotor.set(percent)); }
  public Command Reverse() { return runAtPrecent(-0.75); }
  public Command ReverseArmIntake() {return this.runOnce(()-> ArmIntakeMotor.set(-0.2));}
  public Command stop() { return runAtPrecent(0); }

  @Override
  public void periodic() {
    // logging
    SmartDashboard.putBoolean("intake sensor", armIntakeHasNote.getAsBoolean());
  }
}
