package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class ElevatorConstants{
        public static int MOTOR_ID = 10;

        public static double STAND_POSITION = 10.08;
        public static double SIT_POSITION = 6.08;
        public static double HOME_POSITION = 1.0;

        public static double mmVelo = 2.5;
        public static double mmAcc = 2.5;
        public static double mmJerk = 200;

        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kG = 12.0;
        public static double kP = 2;
        public static double kI = 0.0;
        public static double kD = 0.0;
    }

    public static final class Swerve {
        //* Dimensions & conversions
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );
    }
}
