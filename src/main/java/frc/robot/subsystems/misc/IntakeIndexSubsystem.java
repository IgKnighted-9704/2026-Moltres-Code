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
import frc.robot.subsystems.utility.Sensors;

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
        private boolean autoKick;
    //Data
        private ShuffleboardTab IntakeIndexerSubsystemTab = Shuffleboard.getTab("Intake Indexer Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
        private GenericEntry fuelDetectedIntakeEntry;
        private GenericEntry maxFuelReachedEntry;
        private GenericEntry desiredPivotAngleEntry;
        private GenericEntry currentPivotAngleEntry;
        private GenericEntry intake_kP;
        private GenericEntry intake_kI;
        private GenericEntry intake_kD;
    //Sensors Intialization
        Sensors sensors = new Sensors();

    public IntakeIndexSubsystem(){

        //Initializing Tracker Variables
            fuelDetectedIndexer = false;
            fuelDetectedIntake = false;
            maxFuelReached = false;
            desiredPivotAngle = 0;
            autoKick = true;
        
        //Initializing Shuffleboard Entries
            fuelDetectedIndexerEntry = IntakeIndexerSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
            fuelDetectedIntakeEntry = IntakeIndexerSubsystemTab.add("Fuel Detected Intake", false).getEntry();
            maxFuelReachedEntry = IntakeIndexerSubsystemTab.add("Max Fuel Reached", false).getEntry();
            desiredPivotAngleEntry = IntakeIndexerSubsystemTab.add("Desired Pivot Angle", 0.0).getEntry();
            currentPivotAngleEntry = IntakeIndexerSubsystemTab.add("Current Pivot Angle", 0.0).getEntry();
            intake_kP = IntakeIndexerSubsystemTab.add("INTAKE KP", pivotPID.getP()).getEntry();
            intake_kI = IntakeIndexerSubsystemTab.add("INTAKE KI", pivotPID.getI()).getEntry();
            intake_kD = IntakeIndexerSubsystemTab.add("INTAKE KD", pivotPID.getD()).getEntry();

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
            if(!autoKick){
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
            return Commands.none();
        }

        public Command intakeCommand(){
            return Commands.sequence(
                Commands.runOnce(()->{
                    if(!maxFuelReached && fuelDetectedIntake){
                        setDesired_Angle(Constants.IntakeIndexerSubsystemConstants.INTAKE_ANGLE);
                    }
                }),
                Commands.runOnce(()->{
                    intake();
                })
            );
        }

        public Command stowCommand(){
            return Commands.sequence(
                Commands.runOnce(()->{
                    setDesired_Angle(Constants.IntakeIndexerSubsystemConstants.STOW_ANGLE);
                }),
                Commands.runOnce(()->{
                    stopIntake();
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
        //Update Tracker Variables
            fuelDetectedIndexer = sensors.getIndexSensor();
            fuelDetectedIntake = sensors.getIntakeSensor();
            maxFuelReached = sensors.getHopperLimitSensor();
        //Data
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
            fuelDetectedIntakeEntry.setBoolean(fuelDetectedIntake);
            maxFuelReachedEntry.setBoolean(maxFuelReached);
            desiredPivotAngleEntry.setDouble(desiredPivotAngle);
            currentPivotAngleEntry.setDouble(getIntakeAngle());
        //Change PID Values
            pivotPID.setPID(
                intake_kP.getDouble(pivotPID.getP()), 
                intake_kI.getDouble(pivotPID.getI()), 
                intake_kD.getDouble(pivotPID.getD())
            );
    }
}
