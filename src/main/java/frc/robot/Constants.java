package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

        public static class LightSensor{
            public static final int INTAKE_SENSOR_ID = 0;
            public static final int INDEXER_SENSOR_ID = 0;
            public static final int HOPPER_MAX_LIMIT_SENSOR_ID = 0;
        }

        public static class ClimbSubsystemConstants{
            public static final int CLIMBMOTOR_A = 0;
            public static final int CLIMBMOTOR_B = 0;

            //SPEED CONSTANTS
                public static final double CLIMB_MANUAL_SPEED = 0;
                public static final double CLIMB_MAX_VELOCITY = 0;
                public static final double CLIMB_MAX_ACCELERATION = 0;
            //PID - Climb
                public static double CLIMB_kP = 0;
                public static double CLIMB_kI = 0;
                public static double CLIMB_kD = 0;
            //FEEDFORWARD - Climb
                public static double CLIMB_kS = 0;
                public static double CLIMB_kG = 0;
                public static double CLIMB_kV = 0;
                public static double Climb_kA = 0;

            //HARDWARE CONSTANTS
                public static double CLIMB_WHEEL_RADIUS_INCHES = 0;
            
            //CLIMB HEIGHT CONSTRAINTS
                public static double MAX_INCHES = 0;
                public static double MIN_INCHES = 0;
        
        }

        public static class IntakeIndexerSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 0;
            public static final int INTAKE_PIVOT_MOTOR_ID = 0;
            public static final int INDEXER_MOTOR_ID = 0;

            //SPEED CONSTANTS
                public static final int KICKER_SPEED = 0;
                public static final int INTAKE_SPEED = 0;
                public static final int OUTTAKE_SPEED = 0;
            
            //PID - Angle
                public static double INTAKE_PIVOT_kP = 0;
                public static double INTAKE_PIVOT_kI = 0;
                public static double INTAKE_PIVOT_kD = 0;
            //Angle Macros
                public static double INTAKE_ANGLE = 0;
                public static double STOW_ANGLE = 0;

        }

        public static class ShooterSubsystemConstants{
            public static final int APRILTAG_TARGET_FIDUCIALID = 0;
            public static final int SHOOTER_ANGLE_ID = 0;
            public static final int SHOOTER_ID_A = 0;
            public static final int SHOOTER_ID_B = 0;
            //PID - Angle
                public static double SHOOTER_ANGLE_kP = 0;
                public static double SHOOTER_ANGLE_kI = 0;
                public static double SHOOTER_ANGLE_kD = 0;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0;
                public static double SHOOTER_SPEED_kI = 0;
                public static double SHOOTER_SPEED_kD = 0;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0;
                public static double SHOOTER_SPEED_kV = 0;
                public static double SHOOTER_SPEED_kA = 0;
            //HARDWARE CONSTANTS
                public static double FLYWHEEL_RADIUS_METERS = 0;
                public static double SHOOTER_FLYWHEEL_GEAR_RATIO = 0;
                public static double SHOOTER_ANGLE_OFFSET = 0;
            //SHOOTER ANGLE CONSTRAINTS
                public static double MAX_ANGLE = 0;
                public static double MIN_ANGLE = 0;
                public static double SPEED_TOLERANCE = 0;
                public static double ANGLE_TOLERANCE = 0;
            //PERIODIC CONSTANTS
                public static boolean desiredVelReached = false;
                public static boolean desiredAngleReached = false;
        }

    // Swervedrive Constants
    public static final class SwerveConstants {
        public static final class ModuleConstants {
            // Gear Ratios & Physical Constants
            public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
            public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3 * Math.PI;
            public static final double kDriveMotorGearRatio = 4.71;
            public static final double kAngleMotorGearRatio = 1;

            // Conversion Factors
            public static final double kDriveEncoderRot2Meters = (2 * Math.PI * kWheelDiameterMeters) / kDriveMotorGearRatio;
            public static final double kTurningEncoderRot2Rad = (Math.PI/180) / kAngleMotorGearRatio;
            // PID Values-Angle Motor
            public static double kPTurning = 0.035;
            public static double kITurning = 0.0;
            public static double kDTurning = 0.0;
            // PID Values-Drive Motor
            public static double kPDriving = 1; //1
            public static double kIDriving = 0.0;
            public static double kDDriving = 0.0;
            // Feedforward Values-Drive Motor
            public static double kSDriving = 0;
            public static double kVDriving = 1.1; //1.1
            public static double kADriving = 0.0;
        }

        public static final class DriveConstants {
            // Front Left Module
            public static final int kFrontLeftDriveMotorPort = 2;
            public static final int kFrontLeftTurningMotorPort = 3;
            public static final boolean kFrontLeftDriveEncoderReversed = false;
            public static final boolean kFrontLeftTurningEncoderReversed = false;
            public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.0;
            public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

            // Front Right Module
            public static final int kFrontRightDriveMotorPort = 4;
            public static final int kFrontRightTurningMotorPort = 5;
            public static final boolean kFrontRightDriveEncoderReversed = false;
            public static final boolean kFrontRightTurningEncoderReversed = true;
            public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
            public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

            // Back Left Module
            public static final int kBackLeftDriveMotorPort = 6;
            public static final int kBackLeftTurningMotorPort = 7;
            public static final boolean kBackLeftDriveEncoderReversed = false;
            public static final boolean kBackLeftTurningEncoderReversed = false;
            public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.0;
            public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

            // Back Right Module
            public static final int kBackRightDriveMotorPort = 8;
            public static final int kBackRightTurningMotorPort = 9;
            public static final boolean kBackRightDriveEncoderReversed = false;
            public static final boolean kBackRightTurningEncoderReversed = true;
            public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0;
            public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

            // Gyroscope
            public static final int kGyroPort = 20;

            // Distance Between Wheels Horizontal
            public static final double kTrackWidth = Units.inchesToMeters(24.849);
            // Distance Between Wheels Vertical
            public static final double kWheelBase = Units.inchesToMeters(24.849);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Back Right
            );

            // Auto Constants
                // Translation PID Values
                public static double kPTranslation = 0.0;
                public static double kITranslation = 0.0;
                public static double kDTranslation = 0.0;
                // Rotation PID Values
                public static double kPRotation = 0.0;
                public static double kIRotation = 0.0;
                public static double kDRotation = 0.0;
        }

        public static final class JoyStickConstants {
            public static final double kDeadBand = 0.2;
        }
    }

    public static final class UtilMethods{
        public static boolean checkTolerance(double desireState, double currentState, double tolerance){
            return ((desireState-currentState) < tolerance);
        }

        public static double normalizeDegrees(double degrees) {
            degrees = degrees % 360;

            if (degrees > 180) {
                degrees -= 360;
            } else if (degrees <= -180) {
                degrees += 360;
            }

            return degrees;
        }
    }
}