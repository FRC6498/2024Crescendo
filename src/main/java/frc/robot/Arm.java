package frc.robot;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
  private final TalonFX LeftArmMotor, RightArmMotor; // declare motors that run the arm
  private final CANcoder armCoder; // declare encoder for the arm
  private final ArmFeedforward armFeedforward; // create feedforward to hold arm up while once it reaches target position
  public Arm() {
    // * all units should be in rotations
    super( 
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          // PID controls what the motor should output to go to its set target
            45, // P - Percent - how far from target are you (higher is more aggressive action) 
            0,  // I - Integral - (I have no idea how this works. Maybe increases output bases on how much time it takes to get to setpoint?)
            0.1,// D - Derivative (slope but in calc) - are you getting closer to the target (increase to go to target faster)
            new TrapezoidProfile.Constraints(20 /* max velocity */, 10 /* max acceleration */)));
    LeftArmMotor = new TalonFX(15); // Falcon with CAN id 15
    RightArmMotor = new TalonFX(19); // Falcon with CAN id 19
    armCoder = new CANcoder(44); // CANCoder with id 44
    armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.ARM_KS, // static friction in the system
      Constants.ArmConstants.ARM_KG, // effect of gravity on the system 
      Constants.ArmConstants.ARM_KV  // affect of voltage on the velocity of the system (how fast does it go for a unit voltage)
      // did rough calculations using ReCalc.com
    );

    //reset arm encoder Position to avoid drift between matches (maybe not neccessary)
    armCoder.setPosition(0);
    
  }

  // sets the motor output using the updated position, velocity, and acceleration of the sensors 
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity); // recalculate the feedforward values based on new system state
    // set motor outputs
    LeftArmMotor.setControl(new VoltageOut(output + feedforward)); 
    RightArmMotor.setControl(new VoltageOut(output + feedforward));
    // logging
    SmartDashboard.putNumber("out", output);
    SmartDashboard.putNumber("ff", feedforward);
  }
  
  // get the current position of the arm
  public double getPosition() {return armCoder.getPosition().getValueAsDouble();}

  // update the position of the system
  @Override
  public double getMeasurement() {
    double position = armCoder.getPosition().getValue(); // get latest values from arm encoder
    // logging
    SmartDashboard.putNumber("get arm pos", position);
    SmartDashboard.putBoolean("arm limit", LeftArmMotor.getFault_ReverseHardLimit().getValue());
    return position;
  }
}
