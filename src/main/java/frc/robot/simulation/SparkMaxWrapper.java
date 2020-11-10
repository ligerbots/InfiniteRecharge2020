package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class SparkMaxWrapper extends CANSparkMax{
    private final double BATTERY_VOLTAGE = 13.6; //We are approximating the battery voltage
    private SimDouble m_simSpeed;
    private SimDevice m_simSparkMax;
    public SparkMaxWrapper(int deviceID, MotorType type){
        super(deviceID,type);

        m_simSparkMax = SimDevice.create("SparkMax",deviceID);
        if (m_simSparkMax != null){
            m_simSpeed = m_simSparkMax.createDouble("speed", false, 0.0);
        }
    }

    @Override
    public double get(){
        if (m_simSparkMax != null){
            return m_simSpeed.get();
        }
        return super.get();
    }

    @Override
    public void set(double speed){
        if (m_simSparkMax != null){
            m_simSpeed.set(speed);
        }else{
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (m_simSparkMax != null){
            set(outputVolts/BATTERY_VOLTAGE);
        } else {
            super.setVoltage(outputVolts);
        }
    }
}