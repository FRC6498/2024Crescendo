package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
  DigitalInput climberLowSensor;
  Trigger climberHigh  = new Trigger(()->!climberLowSensor.get());

  private final TalonFX RightClimberMotor;
  public Climber() {
    RightClimberMotor = new TalonFX(14);
    RightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberLowSensor = new DigitalInput(3);

  }
  public Command Run(){
    return this.run(
      ()-> RightClimberMotor.set(0.5));
  }
  public Command Reverse() {
    return this.run(()-> RightClimberMotor.set(-.8)).until(()->RightClimberMotor.getPosition().getValue() < -273).andThen(Stop());
  }
  public Command Stop() {
    return this.runOnce(()-> RightClimberMotor.set(0));
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climber sensor", climberLowSensor.get());
    if (RightClimberMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround) {
      RightClimberMotor.setPosition(0);
    }
  }
}