package frc.robot.subsystems.utility;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
    public final DigitalInput indexSensor = new DigitalInput(Constants.LightSensor.INDEXER_SENSOR_ID);
    public final DigitalInput intakeSensor = new DigitalInput(Constants.LightSensor.INTAKE_SENSOR_ID);
    public final DigitalInput hopperSesnor = new DigitalInput(Constants.LightSensor.HOPPER_MAX_LIMIT_SENSOR_ID);

    public boolean getIndexSensor() {
        return !indexSensor.get();
    }
    public boolean getIntakeSensor(){
        return !intakeSensor.get();
    }
    public boolean getHopperLimitSensor(){
        return !hopperSesnor.get();
    }
}
