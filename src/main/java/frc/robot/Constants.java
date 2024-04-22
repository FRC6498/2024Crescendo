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
        /**
            Position of the camera relative to the middle of the bot
            Transform is in coordinates (x, y, z)
            x is forward/backward
            y is left/right (looking at the front of the bot)
            z is up and down
        */
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(Units.inchesToMeters(11.5), Units.inchesToMeters(0),Units.inchesToMeters(4)), new Rotation3d(0, 0.4, Math.PI));
    }
    public static final class FieldConstants{
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        /** Id of AprilTag on the center of the blue speaker */
        public static final int BLUE_SPEAKER_TAG_ID = 4;
        /** Id of AprilTag on the center of the red speaker */
        public static final int RED_SPEAKER_TAG_ID = 4;
        /** Speaker height in meters */
        public static final int SPEAKER_HEIGHT = 2;
    }
    public static final class ShooterConstants{
        // Top shooter motor sysid constants
        public static final double TOP_MOTOR_KP = 0.00011415;
        public static final double TOP_MOTOR_KI = 0;
        public static final double TOP_MOTOR_KD = 0.0001;
        public static final double TOP_MOTOR_KS = -0.1924;
        public static final double TOP_MOTOR_KV = 0.13174;
        public static final double TOP_MOTOR_KA = 0.07513;
        // Bottom shooter motor sysid constants
        public static final double BOTTOM_MOTOR_KP = 0.0001482;
        public static final double BOTTOM_MOTOR_KI = 0;
        public static final double BOTTOM_MOTOR_KD = 0.0001;
        public static final double BOTTOM_MOTOR_KS = 0.12187;
        public static final double BOTTOM_MOTOR_KV = 0.12947;
        public static final double BOTTOM_MOTOR_KA = 0.032151;
        /**  equation to calculate shooter angle given robot's distance to the speaker */
        public static double CalcShooterAngleFromDistance(double distanceToSpeakerMeters) {
            // tested robot shooting angles and fit a curve to the resulting data
            double val =((-0.0151*Math.pow(distanceToSpeakerMeters, 2)) + (.1*distanceToSpeakerMeters)-.09);
            // added checks for saftey
            if (val > 0.2) {
                return 0.2; // stops the arm to going to far
            }else if(val < 0) {
                return 0; // stops the arm from trying to go down and break the hard stops
            } else {
                return val; // everything is good
            }
        }
        public static final double DEFAULT_SHOOTER_SPEED = 0.75;
    }
    public static final class LedConstants {
        /** total length of leds in the strip on the bot (can be measured with the resistance across the stip) */
        public static final int LED_LENGTH = 120;
    }
    public static final class ArmConstants {
        // PID constants for the arm (manually generated)
        public static final Slot0Configs armConfigs = new Slot0Configs()
        .withKP(3)
        .withKI(0)
        .withKD(0.5);
        // mostly a wild guess with Recalc (Arm feedforward never did very much)
        public static final double ARM_KS = 0;
        public static final double ARM_KG = 0.06;
        public static final double ARM_KV = 2.26;
    }
}
