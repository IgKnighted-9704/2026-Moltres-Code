package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.subsystems.swervesubsystem.SwerveSubsystem;

public class ClimbSubsystem {
    
    //Climb Motor 
        private final TalonFX climbMotorA = new TalonFX(ClimbSubsystemConstants.CLIMBMOTOR_A);
        private final TalonFX climbMotorB = new TalonFX(ClimbSubsystemConstants.CLIMBMOTOR_B);
        private final SwerveSubsystem swerve;
    
    //Tracker Variables
        private double robotPivotAngle;
        private double desiredClimbInches;
    
    //Climb Angle - Profiled PID
        public final ProfiledPIDController climbPID = new ProfiledPIDController(
            ClimbSubsystemConstants.CLIMB_kP, 
            ClimbSubsystemConstants.CLIMB_kI, 
            ClimbSubsystemConstants.CLIMB_kD,
            new TrapezoidProfile.Constraints(ClimbSubsystemConstants.CLIMB_MAX_VELOCITY, ClimbSubsystemConstants.CLIMB_MAX_ACCELERATION)
        );
        public final ElevatorFeedforward climbFF = new ElevatorFeedforward(ClimbSubsystemConstants.CLIMB_kS, ClimbSubsystemConstants.CLIMB_kG, ClimbSubsystemConstants.CLIMB_kV, ClimbSubsystemConstants.Climb_kA);
    //Data
        private final ShuffleboardTab ClimbSubsystemTab = Shuffleboard.getTab("Climb Subsystem Tab");
            private GenericEntry robotPivotAngleEntry;
            private GenericEntry desiredClimbInchesEntry;
            private GenericEntry currentClimbInchesEntry;
        
    public ClimbSubsystem(SwerveSubsystem swerve){
        this.swerve = swerve;
        robotPivotAngle = 0;
            robotPivotAngleEntry = ClimbSubsystemTab.add("Robot Pivot", 0).getEntry();
            desiredClimbInchesEntry = ClimbSubsystemTab.add("Desired Climb Inches", 0).getEntry();
            currentClimbInchesEntry = ClimbSubsystemTab.add("Current Climb Inches", 0).getEntry();
    }

    //Utility Tools
        private double getClimbInches(){
            return 
            ((climbMotorA.getPosition().getValueAsDouble() * (2*Math.PI)
                * Constants.ClimbSubsystemConstants.CLIMB_WHEEL_RADIUS_INCHES) + 
            (climbMotorB.getPosition().getValueAsDouble() * (2*Math.PI)
                * Constants.ClimbSubsystemConstants.CLIMB_WHEEL_RADIUS_INCHES))/2;
        }

    //Subsystem Mthods
        public void climbManual(){
            climbMotorA.set(ClimbSubsystemConstants.CLIMB_MANUAL_SPEED);
            climbMotorB.set(ClimbSubsystemConstants.CLIMB_MANUAL_SPEED);
        }

        public void unclimb(){
            climbMotorA.set(-ClimbSubsystemConstants.CLIMB_MANUAL_SPEED);
            climbMotorB.set(-ClimbSubsystemConstants.CLIMB_MANUAL_SPEED);
        }

        public void setClimbInches(double inches){
            desiredClimbInches = inches;
            climbPID.setGoal(desiredClimbInches);
        }

    public void periodic(){
        robotPivotAngle = swerve.getGyro().getZ();

        //Profiled PID Controller Calculate
            double cPID = climbPID.calculate(getClimbInches());
            double cFF = climbFF.calculate(climbPID.getGoal().velocity);
                climbMotorA.setVoltage(cPID + cFF);
                climbMotorB.setVoltage(cPID + cFF);
        //Data
            robotPivotAngleEntry.setDouble(robotPivotAngle);
            desiredClimbInchesEntry.setDouble(desiredClimbInches);
            currentClimbInchesEntry.setDouble(getClimbInches());
    }

}
