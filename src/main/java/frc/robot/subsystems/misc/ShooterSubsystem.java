package frc.robot.subsystems.misc;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.subsystems.utility.LimelightHelpers;
import frc.robot.subsystems.utility.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.utility.LimelightHelpers.LimelightTarget_Fiducial;

public class ShooterSubsystem extends SubsystemBase{
   
    //Shooter Motor
        private final TalonFX shooterA = new TalonFX(Constants.ShooterSubsystemConstants.SHOOTER_ID_A);
          private final TalonFX shooterB = new TalonFX(Constants.ShooterSubsystemConstants.SHOOTER_ID_B);
        private final SparkMax shooterAngle = new SparkMax(Constants.ShooterSubsystemConstants.SHOOTER_ANGLE_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
            private final SparkAbsoluteEncoder shooterAngleEncoder = shooterAngle.getAbsoluteEncoder();
    
     //Shooter Angle - PID
        private final PIDController shooterAnglePID = new PIDController(
            Constants.ShooterSubsystemConstants.SHOOTER_ANGLE_kP, 
            Constants.ShooterSubsystemConstants.SHOOTER_ANGLE_kI, 
            Constants.ShooterSubsystemConstants.SHOOTER_ANGLE_kD
        );
    //Shooter Speed - PID
        private final PIDController shooterSpeedPID = new PIDController(
            Constants.ShooterSubsystemConstants.SHOOTER_SPEED_kP, 
            Constants.ShooterSubsystemConstants.SHOOTER_SPEED_kI, 
            Constants.ShooterSubsystemConstants.SHOOTER_SPEED_kD
        );
    //Shooter Speed - Feedforward
        private final SimpleMotorFeedforward shooterSpeedFF = new SimpleMotorFeedforward(
            Constants.ShooterSubsystemConstants.SHOOTER_SPEED_kS,
             Constants.ShooterSubsystemConstants.SHOOTER_SPEED_kV
        );

    //Data
        private final ShuffleboardTab ShooterSubsystemTab = Shuffleboard.getTab("Shooter Subsystem Tab");
            private final GenericEntry currentVelEntry;
            private final GenericEntry desiredVelEntry;
            private final GenericEntry currentAngleEntry;
            private final GenericEntry desiredAngleEntry;
            private final GenericEntry targetDistanceEntry;
            private final GenericEntry targetHeightEntry;
            private final GenericEntry subsystemStateEntry;
            private final GenericEntry visionStateEntry;
            private GenericEntry shooterSpeed_kP;
            private GenericEntry shooterSpeed_kI;
            private GenericEntry shooterSpeed_kD;
            private GenericEntry shooterSpeed_kS;
            private GenericEntry shooterSpeed_kV;


    //Tracker Variables
       private boolean enableSubsystem;
       private boolean enableVision;
       private boolean buttonClicked;
       private double desired_Velocity;
       private double desired_Angle;
       private double target_distance;
       private double target_height;

       public ShooterSubsystem(){
            
            //Initializing Tracker Variables
                enableSubsystem = true;
                enableVision = true;
                desired_Velocity = 0.0;
                desired_Angle = 0.0;
                target_distance = 0.0;
                target_height = 0.0;

            //Initializing Shuffleboard Entries
                currentVelEntry = ShooterSubsystemTab.add("Current Velocity", 0.0).getEntry();
                desiredVelEntry = ShooterSubsystemTab.add("Desired Velocity", 0.0).getEntry();
                currentAngleEntry = ShooterSubsystemTab.add("Current Angle", 0.0).getEntry();
                desiredAngleEntry = ShooterSubsystemTab.add("Desired Angle", 0.0).getEntry();
                targetDistanceEntry = ShooterSubsystemTab.add("Target Distance", 0.0).getEntry();
                targetHeightEntry = ShooterSubsystemTab.add("Target Height", 0.0).getEntry();
                subsystemStateEntry = ShooterSubsystemTab.add("Subsystem State", true).getEntry();
                visionStateEntry = ShooterSubsystemTab.add("Vision State", false).getEntry();
                shooterSpeed_kP = ShooterSubsystemTab.add("SHOOTER KP", shooterSpeedPID.getP()).getEntry();
                shooterSpeed_kI = ShooterSubsystemTab.add("SHOOTER KI", shooterSpeedPID.getI()).getEntry();
                shooterSpeed_kD = ShooterSubsystemTab.add("SHOOTER KD", shooterSpeedPID.getD()).getEntry();
                shooterSpeed_kS = ShooterSubsystemTab.add("SHOOTER KS", shooterSpeedFF.getKs()).getEntry();
                shooterSpeed_kV = ShooterSubsystemTab.add("SHOOTER KV", shooterSpeedFF.getKv()).getEntry();
       }

    //Utility Methods
        private double getShooterVelocityMeters(){
            return 
                (shooterA.getVelocity().getValueAsDouble() * (2*Math.PI) 
                    * Constants.ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS + 
                shooterB.getVelocity().getValueAsDouble() * (2*Math.PI) 
                    * Constants.ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS)/2;
        }

        private double getShooterAngleDegrees(){
            return (shooterAngleEncoder.getPosition() - ShooterSubsystemConstants.SHOOTER_ANGLE_OFFSET) * 360;
        }
    //Subsystem Methods
        public void enableSubsystem(){
            enableSubsystem = true;
        }

        public void disableSubsystem(){
            enableSubsystem = false;
        }

        public void enableVisionBasedScoring(){
            enableVision = true;
        }

        public void setDesiredVelocity(double velocity){
            desired_Velocity = velocity;
        }

        public double getDesiredVelocity(){
            return desired_Velocity;
        }

        public void setDesired_Angle(double angle){
            desired_Angle = MathUtil.clamp(angle, Constants.ShooterSubsystemConstants.MIN_ANGLE, Constants.ShooterSubsystemConstants.MAX_ANGLE);
        }
        public double  getDesiredAngle(){
            return desired_Angle;
        }


        //IDEAL PHYSICS, ACCOUNT FOR DRAG LATER
            //θ = arctan(y/x)
            //yVel
                //v^2 - u^2 = 2ax, where v = 0, u = initial velocity along y, a = acceleration, x is target height relative to cam.
                //v(ideal) sin(θ) = u, so v(ideal) = u/sin(θ), so v(ideal) = sqrt(2ax)/sin(θ)
            //xVel 
                //v(ideal) cos(θ) = u
                //(sqrt(2ax)/sin(θ)) cos(θ) = u
            //Overall
                // X Velocity =  (sqrt(2ax)/sin(θ)) cos(θ)
                // Y Velocity = (sqrt(2ax))
                
        public double generateVelocity(){
            double angle = Math.atan2(target_height, target_distance);
            double yVel = Math.sqrt(2 * 9.80665 * target_height);
            double xVel = (angle > 0) ? (Math.sqrt(2 * 9.80665 * target_height)/Math.sin(angle)) * Math.cos(angle) : 0;
                double velocity = Math.hypot(xVel, yVel);
            return velocity;
        }

    //Command Based Methods
        public Command enableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.enableSubsystem();
                    });
        }
        
        public Command disableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.disableSubsystem();
                    });
        }

        public Command setAngleAndVelocityCommand(double angle, double velocity){
            if(!enableVision){
                return Commands.runOnce(()->{
                        this.setDesired_Angle(angle);
                        this.setDesiredVelocity(velocity);
                });
            }
            return Commands.none();
        }

    @Override
    public void periodic(){
        if(enableSubsystem){
            //Shooter Speed 
                double sPID = shooterSpeedPID.calculate(getShooterVelocityMeters(), desired_Velocity);
                double sFF = shooterSpeedFF.calculate(desired_Velocity);
                    shooterA.setVoltage(MathUtil.clamp(sPID + sFF, -12, 12));
                    shooterB.setVoltage(-MathUtil.clamp(sPID + sFF, -12, 12));
            //Shooter Angle
                double anglePID = shooterAnglePID.calculate(getShooterAngleDegrees(), desired_Angle);
                    shooterAngle.setVoltage(MathUtil.clamp(anglePID, -6, 6));
            //Set Global Constants
                Constants.ShooterSubsystemConstants.desiredVelReached = 
                    Constants.UtilMethods.checkTolerance(desired_Velocity, getShooterVelocityMeters(), Constants.ShooterSubsystemConstants.SPEED_TOLERANCE);
                Constants.ShooterSubsystemConstants.desiredAngleReached = 
                    Constants.UtilMethods.checkTolerance(desired_Angle, getShooterAngleDegrees(), Constants.ShooterSubsystemConstants.ANGLE_TOLERANCE);
            //Vision
            if(enableVision){
                boolean foundTarget = false;
                LimelightResults results = LimelightHelpers.getLatestResults("limeLight");
                if(results.targets_Fiducials.length > 0){
                    for(LimelightTarget_Fiducial tag : results.targets_Fiducials){
                        if(tag.fiducialID == Constants.ShooterSubsystemConstants.APRILTAG_TARGET_FIDUCIALID){
                            Pose3d targetPose = tag.getTargetPose_CameraSpace();
                                target_distance = targetPose.getX();
                                target_height = targetPose.getZ();
                                foundTarget = true;
                                break;
                            }
                        }
                    }
                if(foundTarget){
                  setDesired_Angle(Units.radiansToDegrees(Math.atan2(target_height, target_distance)));
                  setDesiredVelocity(generateVelocity());
                }
            }
            //Data
                currentVelEntry.setDouble(getShooterVelocityMeters());
                desiredVelEntry.setDouble(desired_Velocity);
                currentAngleEntry.setDouble(getShooterAngleDegrees());
                desiredAngleEntry.setDouble(desired_Angle);
                targetDistanceEntry.setDouble(target_distance);
                targetHeightEntry.setDouble(target_height);
                subsystemStateEntry.setBoolean(enableSubsystem);
                visionStateEntry.setBoolean(enableVision);
            //Change PID Values
                shooterSpeedPID.setPID(shooterSpeed_kP.getDouble(shooterSpeedPID.getP()), shooterSpeed_kI.getDouble(shooterSpeedPID.getI()), shooterSpeed_kD.getDouble(shooterSpeedPID.getD()));
            //Change Feedforward Values
                shooterSpeedFF.setKs(shooterSpeed_kS.getDouble(shooterSpeedFF.getKs()));
                shooterSpeedFF.setKv(shooterSpeed_kV.getDouble(shooterSpeedFF.getKv()));
        }

    }
}
