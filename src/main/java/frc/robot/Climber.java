package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  DigitalInput climberSensor;
  private final TalonFX LeftClimberMotor, RightClimberMotor;
  public Climber() {
    LeftClimberMotor = new TalonFX(20);
    RightClimberMotor = new TalonFX(14);
    climberSensor = new DigitalInput(3);
  }
  public Command Run(){
    return this.run(
      ()-> LeftClimberMotor.set(0.5)).until(()-> climberSensor.get())
      .andThen(()-> RightClimberMotor.set(0.5))
      .until(()-> climberSensor.get());
  }
  public Command Reverse() {
    return this.runOnce(()-> RightClimberMotor.set(-1)).andThen(()-> LeftClimberMotor.set(-1));
  }
  public Command Stop() {
    return this.runOnce(()-> LeftClimberMotor.set(0)).andThen(()-> RightClimberMotor.set(0));
  }
  @Override
  public void periodic() {
  }
}