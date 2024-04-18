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
  DigitalInput climberLowSensor; // Declare sensor that detects when the climber gets to low 
  Trigger climberHigh  = new Trigger(()->!climberLowSensor.get() /* sensor is inverted */); // Trigger that updates when the climber gets to low  

  private final TalonFX RightClimberMotor;

  public Climber() {
    RightClimberMotor = new TalonFX(14);
    RightClimberMotor.setNeutralMode(NeutralModeValue.Brake); // set the climber to brake when no output is being supplied
    climberLowSensor = new DigitalInput(3);
  }

  // moves the climber down (motor runs forward )
  public Command Run() {
    return this.run(
      ()-> RightClimberMotor.set(1));
  }
  // moves the climber up (motor runs backward)
  public Command Reverse() {
    return this.run(()-> RightClimberMotor.set(-1)).until(()->RightClimberMotor.getPosition().getValue() < -273).andThen(Stop());
  }
  // stops the arm
  public Command Stop() {
    return this.runOnce(()-> RightClimberMotor.set(0));
  }

  @Override // by default the periodic method in a subsystem is empty so it has to be overridden for any of the code below to work
  public void periodic() {
    // log climber sensor state
    SmartDashboard.putBoolean("climber sensor", climberLowSensor.get());

    // reset climber motor integrated encoder position to 0 if climber reaches low point
    if (RightClimberMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround) {
      RightClimberMotor.setPosition(0);
    }
  }
}