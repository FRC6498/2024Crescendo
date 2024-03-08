package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    Vision vision;


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        vision = new Vision();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        vision = new Vision();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    private void configurePathPlanner() {
       double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->false, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    public double GetDistanceToSpeaker() {
        // dist from robot pose to speaker pose
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return m_odometry.getEstimatedPosition().getTranslation().getDistance(Constants.FieldConstants.FIELD_LAYOUT.getTagPose(Constants.FieldConstants.BLUE_SPEAKER_TAG_ID).get().toPose2d().getTranslation());
        }else {
            return m_odometry.getEstimatedPosition().getTranslation().getDistance(Constants.FieldConstants.FIELD_LAYOUT.getTagPose(Constants.FieldConstants.RED_SPEAKER_TAG_ID).get().toPose2d().getTranslation());
        }
    }

    public Rotation2d getRobotToSpeakerRotation() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return Constants.FieldConstants.FIELD_LAYOUT.getTagPose(Constants.FieldConstants.BLUE_SPEAKER_TAG_ID).get().toPose2d().getRotation().minus(m_odometry.getEstimatedPosition().getRotation());
        }else{
            //field at castle is setup as red
            return Constants.FieldConstants.FIELD_LAYOUT.getTagPose(Constants.FieldConstants.RED_SPEAKER_TAG_ID).get().toPose2d().getRotation().minus(m_odometry.getEstimatedPosition().getRotation());
        }
    }

    public void periodic() {
        var visionEst = vision.updatePoseEstimator();
        if (visionEst.isPresent()){
            m_odometry.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), vision.getCurrentTimeStamp());
        }
        // SmartDashboard.putNumber("SpeakerRotation", getRobotToSpeakerRotation().getDegrees());
    }

}
