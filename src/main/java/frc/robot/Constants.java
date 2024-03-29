package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public static final class VisionConstants{
       public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(Units.inchesToMeters(11.5), Units.inchesToMeters(0),Units.inchesToMeters(4)), new Rotation3d(0, 0.4, Math.PI));
    }
    public static final class FieldConstants{
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final int BLUE_SPEAKER_TAG_ID = 4;
        public static final int RED_SPEAKER_TAG_ID = 4;
        public static final int SPEAKER_HEIGHT = 2; // speaker height in m
    }
    public static final class IntakeConstants{
        /**Speed that the intake will run at unless directly overridden by the runAtPercent command */
        public static final double DEFAULT_INTAKE_SPEED = 0.5;
    }
    public static final class ShooterConstants{
        //Top shooter sysid constants
        public static final double TOP_MOTOR_KP = 0.00011415;
        public static final double TOP_MOTOR_KI = 0;
        public static final double TOP_MOTOR_KD = 0.0001;
        public static final double TOP_MOTOR_KS = -0.1924;
        public static final double TOP_MOTOR_KV = 0.13174;
        public static final double TOP_MOTOR_KA = 0.07513;
        //Bottom shooter motor sysid constants
        public static final double BOTTOM_MOTOR_KP = 0.0001482;
        public static final double BOTTOM_MOTOR_KI = 0;
        public static final double BOTTOM_MOTOR_KD = 0.0001;
        public static final double BOTTOM_MOTOR_KS = 0.12187;
        public static final double BOTTOM_MOTOR_KV = 0.12947;
        public static final double BOTTOM_MOTOR_KA = 0.032151;
        public static double CalcShooterAngleFromDistance(double distanceToSpeakerMeters) {
            double val =((-0.0151*Math.pow(distanceToSpeakerMeters, 2)) + (.1*distanceToSpeakerMeters)-.09); 
            if (val > 0.2) {
                return 0.2;
            }else if(val < 0) {
                return 0;
            } else {
                return val;
            }
        }
        public static final double DEFAULT_SHOOTER_SPEED = 0.75;
    }
    public static final class LedConstants {
        public static final int LED_LENGTH = 120;
    }
    public static final class ArmConstants {
        // TODO: config arm constants
        public static final Slot0Configs armConfigs = new Slot0Configs()
        .withKP(3)
        .withKI(0)
        .withKD(0.5);
        public static final double ARM_KS = 0;
        public static final double ARM_KG = 0.06;
        public static final double ARM_KV = 2.26;
    }
}
