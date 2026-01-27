package frc.robot.subsystems.swervesubsystem;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
public class SwerveModule{

    //Drive & Angle Motors
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    
    //Drive & Angle Encoders
    private final SparkAbsoluteEncoder angleEncoder;

    //Tracker
    public double targetAngle;
    public double error;

    private final boolean angleMotorReversed;
    private final double angleMotorOffsetRot;
    private final boolean driveMotorReversed;

    private PIDController anglePIDController;
    private PIDController drivePIDController;
    private SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed, double angleMotorOffsetRot){
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        this.angleMotorOffsetRot = angleMotorOffsetRot;

        this.angleEncoder = angleMotor.getAbsoluteEncoder(); //Spark Absolute Encoder - Neo

        this.angleMotorReversed = angleMotorReversed;
        this.driveMotorReversed = driveMotorReversed;

        //Drive PID Initialization
        drivePIDController = new PIDController(Constants.SwerveConstants.ModuleConstants.kPDriving, 
                                              Constants.SwerveConstants.ModuleConstants.kIDriving,
                                              Constants.SwerveConstants.ModuleConstants.kDDriving);
        //Drive Feedforward Initialization
        driveFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.ModuleConstants.kSDriving, 
                                                     Constants.SwerveConstants.ModuleConstants.kVDriving, 
                                                     Constants.SwerveConstants.ModuleConstants.kADriving);
        //Angle PID Initialization 
        anglePIDController = new PIDController(Constants.SwerveConstants.ModuleConstants.kPTurning, 
                                               Constants.SwerveConstants.ModuleConstants.kITurning,
                                               Constants.SwerveConstants.ModuleConstants.kDTurning);

        //Rather then using the max and min input range as constraints, it considers them to be the same point and automatically calculates the shortest route to the setpoint.
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
     targetAngle = 0;
    }

    public double getDrivePosition(){
        return driveMotorReversed ? driveMotor.getPosition().getValueAsDouble() * -1 * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters :
                                   driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters;
    }

    public double getAngularPosition(){
        return (angleEncoder.getPosition() - angleMotorOffsetRot) * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity(){
        return driveMotorReversed ? driveMotor.getVelocity().getValueAsDouble() * -1 * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters :
                                   driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters;
    }

    public double getAngularVelocity(){
        return 
        angleEncoder.getVelocity() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad;
    }

   public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngularPosition()));
   }

   public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAngularPosition()));
   }

   public PIDController getDrivePID(){
        return drivePIDController; 
   }

   public PIDController getAngularPID(){
        return anglePIDController;
   }

   public SimpleMotorFeedforward getSimpleMotorFeedforward(){
        return driveFeedforward;
   }

   public void setDrivePID(double kP, double kI, double kD){
        drivePIDController.setPID(kP, kI, kD);
   }

   public void setAngularPID(double kP, double kI, double kD){
        anglePIDController.setPID(kP, kI, kD);
   }
   public void setDriveFeedForward(double kS, double kV, double kA){
        driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
   }

    public void resetEncoders(){
        driveMotor.setPosition(0);
    }
   public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state.optimize(getState().angle);
            double totalSpeed = drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond) + 
                                driveFeedforward.calculate(state.speedMetersPerSecond);
            double targetAngle = Math.IEEEremainder(state.angle.getRadians(), Math.PI);
            this.targetAngle = targetAngle;
        driveMotor.setVoltage(totalSpeed);
     //    angleMotor.setVoltage(anglePIDController.calculate(getAngularPosition(),targetAngle));
        double output = anglePIDController.calculate(getAngularPosition(),targetAngle);
        error = Math.abs(targetAngle)  - Math.abs(Math.IEEEremainder(getAngularPosition(), Math.PI));
        if(Math.abs(error) < Math.toRadians(10)){
          angleMotor.setVoltage(0);
        } else {
          angleMotor.set(output);
        }
   }

   public void stop(){
        driveMotor.setVoltage(0);
        angleMotor.set(0);
   }

}
