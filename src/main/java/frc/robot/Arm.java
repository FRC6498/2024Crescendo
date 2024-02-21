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
  private final TalonFX LeftArmMotor, RightArmMotor;
  private final ArmFeedforward armFeedforward;
  private final CANcoder armCoder;
  public final double inchesToArmDegress = 8.33e-4;
  // private final SysIdRoutine armIdRoutine;
  // private final MutableMeasure<Voltage> RightAppliedVoltage, LeftAppliedVoltage;
  // private final MutableMeasure<Angle> RightAngle, LeftAngle;
  // private final MutableMeasure<Velocity<Angle>> RightVelocity, LeftVelocity;
  public Arm() {
    // * all units should be in rotations
    super( 
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            45,
            0, //! arm needs more tuning to make it freak out less
            0.1,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(20, 10)));
    LeftArmMotor = new TalonFX(15);
    RightArmMotor = new TalonFX(19);
    armCoder = new CANcoder(44);
    armFeedforward = new ArmFeedforward(Constants.ArmConstants.ARM_KS, Constants.ArmConstants.ARM_KG, Constants.ArmConstants.ARM_KV);
    armCoder.setPosition(0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    LeftArmMotor.setControl(new VoltageOut(output + feedforward));
    RightArmMotor.setControl(new VoltageOut(output + feedforward));
    SmartDashboard.putNumber("out", output);
    SmartDashboard.putNumber("ff", feedforward);
  }
  public double getPosition() {return armCoder.getPosition().getValueAsDouble();}

  @Override
  public double getMeasurement() {
    double position = armCoder.getPosition().getValue();
    SmartDashboard.putNumber("get arm pos", position);
    return position;
  }
}
