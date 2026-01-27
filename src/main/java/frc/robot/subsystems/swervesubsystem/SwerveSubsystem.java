package frc.robot.subsystems.swervesubsystem;

import java.io.IOException;
import java.util.Set;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UtilMethods;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.subsystems.utility.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
	private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    public final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private Pigeon2 gyroscope;

    private  LimelightHelpers.PoseEstimate limelightMeasurement;
    private final SwerveDrivePoseEstimator poseEstimator;

    public final ShuffleboardTab SwerveSubsystemTracker;
        private GenericEntry RobotVelocity;
        private GenericEntry RobotHeading;
        
        private GenericEntry FrontRightModuleDriveVelocity;
        private GenericEntry FrontRightModuleAngle;
        private GenericEntry FrontLeftModuleDriveVelocity;
        private GenericEntry FrontLeftModuleAngle;
        private GenericEntry BackRightModuleDriveVelocity;
        private GenericEntry BackRightModuleAngle;
        private GenericEntry BackLeftModuleDriveVelocity;
        private GenericEntry BackLeftModuleAngle;
        
        private GenericEntry PoseEstimatorX;
        private GenericEntry PoseEstimatorY;
        private GenericEntry PoseEstimatorRotation;

        private GenericEntry PathPlannerTranslationkP;
        private GenericEntry PathPlannerTranslationkI;
        private GenericEntry PathPlannerTranslationkD;

        private GenericEntry PathPlannerRotationkP;
        private GenericEntry PathPlannerRotationkI;
        private GenericEntry PathPlannerRotationKd;

        private GenericEntry DrivekP;
        private GenericEntry DrivekI;
        private GenericEntry DrivekD;

        private GenericEntry AnglekP;
        private GenericEntry AnglekI;
        private GenericEntry AnglekD;

        private GenericEntry DriveFFkS;
        private GenericEntry DriveFFkV;
        private GenericEntry DriveFFkA;

        //tracker
        private GenericEntry error;

    public SwerveSubsystem() {

        //Module Initialization
        frontLeftModule = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            0.0
        );

        frontRightModule = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            0.0
        );

        backLeftModule = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            0.0
        );

        backRightModule = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            0.0
        );

        //Gyroscope Initializaiton
        gyroscope = new Pigeon2(Constants.SwerveConstants.DriveConstants.kGyroPort);

        //Zero Gyroscope at Startup
        new Thread(()->{
            try {
                Thread.sleep(1000);
                zeroGyroscope();
            } catch (Exception e) {
            }
        }).start();

        // Initial Module Positions
            SwerveModulePosition [] initialModulePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            };
        
        //Pose Estimator Setup
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DriveConstants.kDriveKinematics, getRotation2d(), initialModulePositions , new Pose2d(0, 0, new Rotation2d(0)));

        //Shuffleboard Initialization
        SwerveSubsystemTracker = Shuffleboard.getTab("Swerve Subsystem");
            //Robot Information
                RobotVelocity = SwerveSubsystemTracker.add("Robot Velocity", Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond)).getEntry();
                RobotHeading = SwerveSubsystemTracker.add("Robot Heading", getHeading()).getEntry();
            //Module Information
                FrontRightModuleDriveVelocity = SwerveSubsystemTracker.add("Front Right Module Drive Velocity", frontRightModule.getDriveVelocity()).getEntry();
                FrontRightModuleAngle = SwerveSubsystemTracker.add("FronModule Angle", Math.toDegrees(frontRightModule.getAngularPosition())).getEntry();
                FrontLeftModuleDriveVelocity = SwerveSubsystemTracker.add("Front Left Module Drive Velocity", frontLeftModule.getDriveVelocity()).getEntry();
                FrontLeftModuleAngle = SwerveSubsystemTracker.add("Front Left Module Angle", Math.toDegrees(frontLeftModule.getAngularPosition())).getEntry();
                BackRightModuleDriveVelocity = SwerveSubsystemTracker.add("Back Right Module Drive Velocity", backRightModule.getDriveVelocity()).getEntry();
                BackRightModuleAngle = SwerveSubsystemTracker.add("Back Right Module Angle", Math.toDegrees(backRightModule.getAngularPosition())).getEntry();
                BackLeftModuleDriveVelocity = SwerveSubsystemTracker.add("Back Left Module Drive Velocity", backLeftModule.getDriveVelocity()).getEntry();
                BackLeftModuleAngle = SwerveSubsystemTracker.add("Back Left Module Angle", Math.toDegrees(backLeftModule.getAngularPosition())).getEntry();            
            //Pose Estimator
                PoseEstimatorX = SwerveSubsystemTracker.add("Pose Estimator X", getPose().getX()).getEntry();
                PoseEstimatorY = SwerveSubsystemTracker.add("Pose Estimator Y", getPose().getY()).getEntry();
                PoseEstimatorRotation = SwerveSubsystemTracker.add("Pose Estimator Z", getPose().getRotation().getDegrees()).getEntry();
                error = SwerveSubsystemTracker.add("error", backLeftModule.error).getEntry();
            //Pathplanner
                //Translation
                PathPlannerTranslationkP = SwerveSubsystemTracker
                    .add("Path Planner kP", Constants.SwerveConstants.DriveConstants.kPTranslation)
                    .getEntry();
                PathPlannerTranslationkI = SwerveSubsystemTracker
                    .add("Path Planner kI", Constants.SwerveConstants.DriveConstants.kITranslation)
                    .getEntry();
                PathPlannerTranslationkD = SwerveSubsystemTracker
                    .add("Path Planner kD", Constants.SwerveConstants.DriveConstants.kDTranslation)
                    .getEntry();
                //Rotation
                PathPlannerRotationkP = SwerveSubsystemTracker
                    .add("Rotation kP", Constants.SwerveConstants.DriveConstants.kPRotation)
                    .getEntry();
                PathPlannerRotationkI = SwerveSubsystemTracker
                    .add("Rotation kI", Constants.SwerveConstants.DriveConstants.kIRotation)
                    .getEntry();
                PathPlannerRotationKd = SwerveSubsystemTracker
                    .add("Rotation kD", Constants.SwerveConstants.DriveConstants.kDRotation)
                    .getEntry();
            //Raw Swerve
                // Drive PID
                DrivekP = SwerveSubsystemTracker
                    .add("Drive kP", Constants.SwerveConstants.ModuleConstants.kPDriving)
                    .getEntry();
                DrivekI = SwerveSubsystemTracker
                    .add("Drive kI", Constants.SwerveConstants.ModuleConstants.kIDriving)
                    .getEntry();
                DrivekD = SwerveSubsystemTracker
                    .add("Drive kD", Constants.SwerveConstants.ModuleConstants.kDDriving)
                    .getEntry();

                // Angular PID (Turning Motor)
                AnglekP = SwerveSubsystemTracker
                    .add("Turn kP", Constants.SwerveConstants.ModuleConstants.kPTurning)
                    .getEntry();
                AnglekI = SwerveSubsystemTracker
                    .add("Turn kI", Constants.SwerveConstants.ModuleConstants.kITurning)
                    .getEntry();
                AnglekD = SwerveSubsystemTracker
                    .add("Turn kD", Constants.SwerveConstants.ModuleConstants.kDTurning)
                    .getEntry();

                // Drive Feedforward
                DriveFFkS = SwerveSubsystemTracker
                    .add("Drive kS", Constants.SwerveConstants.ModuleConstants.kSDriving)
                    .getEntry();
                DriveFFkV = SwerveSubsystemTracker
                    .add("Drive kV", Constants.SwerveConstants.ModuleConstants.kVDriving)
                    .getEntry();
                DriveFFkA = SwerveSubsystemTracker
                    .add("Drive kA", Constants.SwerveConstants.ModuleConstants.kADriving)
                    .getEntry();
        setupPathPlanner();
    }

    //Utility Methods
        public static ChassisSpeeds toChassisSpeeds(double vx, double vy, double omega, boolean fieldRelative, SwerveSubsystem swerveSubsystem){
            return fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega , swerveSubsystem.getRotation2d()) : 
                                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega , swerveSubsystem.getRotation2d()) ;
        }

    //Swerve Data Acess
        //Gyroscope
        public Rotation3d getGyro(){
            return gyroscope.getRotation3d();
        }
        //Gyroscope Heading
        public double getHeading(){
                return Math.IEEEremainder(gyroscope.getYaw().getValueAsDouble(), 360);
        }
        //Gyroscope Heading - Rotation2D
        public Rotation2d getRotation2d(){
            return Rotation2d.fromDegrees(getHeading());
        }
        //PoseEstimator - Robot Pose
        public Pose2d getPose(){
            return poseEstimator.getEstimatedPosition();
        }
        //Robot Velocity
        public ChassisSpeeds getRobotVelocity(){
            return Constants.SwerveConstants.DriveConstants.kDriveKinematics.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
            );
        } 
        //Module Positions
        public SwerveModulePosition [] getModulePositions(){
            return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            };
        }
        //Module States
        public SwerveModuleState [] getModuleStates(){
            return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
            };
        }
    //Swerve Methods
        //Set Robot Speed
        public void setModuleStates(SwerveModuleState[] desiredStates, double rotSpeed){
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
            if(Math.abs(rotSpeed) > 0){
                desiredStates[0].angle = new Rotation2d(Math.toRadians(45));
                desiredStates[0].speedMetersPerSecond = -rotSpeed;
                desiredStates[1].angle = new Rotation2d(Math.toRadians(45));
                desiredStates[1].speedMetersPerSecond = rotSpeed;
                desiredStates[2].angle = new Rotation2d(Math.toRadians(135));
                desiredStates[2].speedMetersPerSecond = rotSpeed;
                desiredStates[3].angle = new Rotation2d(Math.toRadians(45));
                desiredStates[3].speedMetersPerSecond = rotSpeed;
                frontLeftModule.setDesiredState(desiredStates[0]);
                frontRightModule.setDesiredState(desiredStates[1]);
                backLeftModule.setDesiredState(desiredStates[2]);
                backRightModule.setDesiredState(desiredStates[3]);
            } else {
                frontLeftModule.setDesiredState(desiredStates[0]);
                frontRightModule.setDesiredState(desiredStates[1]);
                backLeftModule.setDesiredState(desiredStates[2]);
                backRightModule.setDesiredState(desiredStates[3]);
            }
        }
        //Stop Robot
        public void stopModules(){
            frontLeftModule.stop();
            frontRightModule.stop();
            backLeftModule.stop();
            backRightModule.stop();
        }
        //Zero Gyro
        public void zeroGyroscope(){
            gyroscope.reset();
        }

        //Pathplanner
            //Setup Pathplanner
            public void setupPathPlanner(){
                RobotConfig config;
                try {
                    config = RobotConfig.fromGUISettings();
                    AutoBuilder.configure(
                        () -> this.getPose(), 
                        pose -> this.poseEstimator.resetPosition(
                            getRotation2d(), 
                            new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition()
                            },
                            pose) , 
                            () -> this.getRobotVelocity(), 
                            (speeds, feedforwards) -> this.setModuleStates(
                                SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
                                    toChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, this)
                                ),0
                            ), 
                            new PPHolonomicDriveController( 
                                    new PIDConstants(
                                        SwerveConstants.DriveConstants.kPTranslation, 
                                        SwerveConstants.DriveConstants.kITranslation, 
                                        SwerveConstants.DriveConstants.kDTranslation
                                    ),
                                    new PIDConstants(
                                        SwerveConstants.DriveConstants.kPRotation, 
                                        SwerveConstants.DriveConstants.kIRotation, 
                                        SwerveConstants.DriveConstants.kDRotation
                                    ) 
                                    
                            ),
                            config, 
                            () -> {
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
                            }
                            return false;
                            },
                            this
                    );
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }

            }

    @Override
    public void periodic() {
        //Update Pose Estimator
        poseEstimator.update(getRotation2d(), 
               this.getModulePositions()
        );
        
         // First, tell Limelight your robot's current orientation
  double robotYaw = gyroscope.getYaw().getValueAsDouble();
  LimelightHelpers.SetRobotOrientation("limeLight", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Get the pose estimate
    if(DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)){
       limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limeLight");
    } else {
       limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limeLight");
    }
    
  // Update pose estimator with odometry
      poseEstimator.update(
          gyroscope.getRotation2d(),
          this.getModulePositions()
      );

  // Add it to your pose estimator
      if(limelightMeasurement != null && limelightMeasurement.tagCount > 0){
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, Math.toRadians(5)));
            poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
      );
    }

        //Shuffleboard
            //Robot Information
                RobotVelocity.setDouble(Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond));
                RobotHeading.setDouble(getHeading());
            //Module Information
                FrontRightModuleDriveVelocity.setDouble(frontRightModule.getDriveVelocity());
                FrontRightModuleAngle.setDouble(Math.IEEEremainder(frontRightModule.getAngularPosition(), Math.PI));
                FrontLeftModuleDriveVelocity.setDouble(frontLeftModule.getDriveVelocity());
                FrontLeftModuleAngle.setDouble(Math.IEEEremainder(frontLeftModule.getAngularPosition(), Math.PI));
                BackRightModuleDriveVelocity.setDouble(backRightModule.getDriveVelocity());
                BackRightModuleAngle.setDouble(Math.IEEEremainder(backRightModule.getAngularPosition(), Math.PI));
                BackLeftModuleDriveVelocity.setDouble(backLeftModule.getDriveVelocity());
                BackLeftModuleAngle.setDouble(Math.IEEEremainder(backLeftModule.getAngularPosition(), Math.PI));
            //Pose Estimator
                PoseEstimatorX.setDouble(getPose().getX());
                PoseEstimatorY.setDouble(getPose().getY());
                PoseEstimatorRotation.setDouble(getPose().getRotation().getDegrees());
                error.setDouble(backLeftModule.error);
    }
}