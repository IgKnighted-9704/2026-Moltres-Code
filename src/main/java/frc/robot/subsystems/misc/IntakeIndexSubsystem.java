package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndexSubsystem extends SubsystemBase {
    
    //Intake Motor
        private final SparkMax intakeMotor = new SparkMax(Constants.IntakeIndexerSubsystemConstants.INTAKE_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final TalonFX intakePivot = new TalonFX(Constants.IntakeIndexerSubsystemConstants.INTAKE_PIVOT_MOTOR_ID);
        private final TalonFX kickerMotor = new TalonFX(Constants.IntakeIndexerSubsystemConstants.INDEXER_MOTOR_ID);

    //PID - Intake Pivot
        private final PIDController pivotPID = new PIDController(Constants.IntakeIndexerSubsystemConstants.INTAKE_PIVOT_kP, Constants.IntakeIndexerSubsystemConstants.INTAKE_PIVOT_kI, Constants.IntakeIndexerSubsystemConstants.INTAKE_PIVOT_kD);

    //Tracker Variables
        private boolean fuelDetectedIntake;
        private boolean fuelDetectedIndexer;
        private boolean maxFuelReached;
        private double desiredPivotAngle;
    //Data
        private ShuffleboardTab IntakeIndexerSubsystemTab = Shuffleboard.getTab("Intake Indexer Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
        private GenericEntry fuelDetectedIntakeEntry;
        private GenericEntry maxFuelReachedEntry;
        private GenericEntry desiredPivotAngleEntry;
        private GenericEntry currentPivotAngleEntry;

    public IntakeIndexSubsystem(){

        //Initializing Tracker Variables
            fuelDetectedIndexer = false;
            fuelDetectedIntake = false;
            maxFuelReached = false;
            desiredPivotAngle = 0;
        
        //Initializing Shuffleboard Entries
            fuelDetectedIndexerEntry = IntakeIndexerSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
            fuelDetectedIntakeEntry = IntakeIndexerSubsystemTab.add("Fuel Detected Intake", false).getEntry();
            maxFuelReachedEntry = IntakeIndexerSubsystemTab.add("Max Fuel Reached", false).getEntry();
            desiredPivotAngleEntry = IntakeIndexerSubsystemTab.add("Desired Pivot Angle", 0.0).getEntry();
            currentPivotAngleEntry = IntakeIndexerSubsystemTab.add("Current Pivot Angle", 0.0).getEntry();
    }

    //Utility Methods
        public double getIntakeAngle(){
            return intakePivot.getPosition().getValueAsDouble() * 360;
        }
    //Subsystem Methods
        public void intake() {
            if(!maxFuelReached && fuelDetectedIntake){
                intakeMotor.set(Constants.IntakeIndexerSubsystemConstants.INTAKE_SPEED);
            }
        }

        public void outtake(){
            intakeMotor.set(Constants.IntakeIndexerSubsystemConstants.OUTTAKE_SPEED);
        }

        public void stopIntake(){
            intakeMotor.set(0);
        }

        public void setDesired_Angle(double angle){
            desiredPivotAngle = angle;
        }

    //Command Based Methods
        
        public Command kick(){
            return Commands.sequence(
                Commands.runOnce(()->{
                     if(fuelDetectedIndexer && (Constants.ShooterSubsystemConstants.desiredAngleReached && Constants.ShooterSubsystemConstants.desiredVelReached)){
                        kickerMotor.set(Constants.IntakeIndexerSubsystemConstants.KICKER_SPEED);
                    }
                }),
                Commands.waitUntil(()->!fuelDetectedIndexer),
                Commands.runOnce(()->{
                    kickerMotor.set(0);
                })
            );
        }

        public Command intakeCommand(){
            return Commands.parallel(
                Commands.runOnce(()->{
                    intake();
                }),
                Commands.runOnce(()->{
                    if(!maxFuelReached && fuelDetectedIntake){
                        setDesired_Angle(Constants.IntakeIndexerSubsystemConstants.INTAKE_ANGLE);
                    }
                })
            );
        }

        public Command stowCommand(){
            return Commands.parallel(
                Commands.runOnce(()->{
                    stopIntake();
                }),
                Commands.runOnce(()->{
                    setDesired_Angle(Constants.IntakeIndexerSubsystemConstants.STOW_ANGLE);
                })
            );
        }
        
        public Command outtakeCommand(){
            return Commands.runOnce(()->
                this.outtake()
            );
        }
    @Override 
    public void periodic(){
        //Pivot Angle
            double pPID = pivotPID.calculate(getIntakeAngle(), desiredPivotAngle);
                intakePivot.setVoltage(pPID);
        //Indexer
            if(fuelDetectedIndexer && (Constants.ShooterSubsystemConstants.desiredAngleReached && Constants.ShooterSubsystemConstants.desiredVelReached)){
                kickerMotor.set(Constants.IntakeIndexerSubsystemConstants.KICKER_SPEED);
            } else {
                kickerMotor.set(0);
            }
        //Data
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
            fuelDetectedIntakeEntry.setBoolean(fuelDetectedIntake);
            maxFuelReachedEntry.setBoolean(maxFuelReached);
            desiredPivotAngleEntry.setDouble(desiredPivotAngle);
            currentPivotAngleEntry.setDouble(getIntakeAngle());
    }
}
