/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {
    private final WPI_TalonSRX spinner;
    private final Encoder carouselEncoder;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 ballSensor = new ColorSensorV3(i2cPort);

    public Carousel() {
        spinner = new WPI_TalonSRX(Constants.CAROUSEL_CAN_ID);
        spinner.setNeutralMode(NeutralMode.Brake);
        carouselEncoder = new Encoder(5, 6);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (ballSensor.getColor() == ColorMatch.makeColor(0.361, 0.524, 0.113))
            SmartDashboard.putString("Ball? ", "Yes");
        else
            SmartDashboard.putString("Ball? ", "No");
        SmartDashboard.putNumber("Proximity value: ", ballSensor.getProximity());
    }

    public void spin(double speed) {
        spinner.set(ControlMode.PercentOutput, speed);
    }

    public double getCurrent() {
        return spinner.getStatorCurrent();
    }

    public void warmUp() {
        this.spin(Constants.CAROUSEL_SHOOTER_SPEED);
    }

    public int getTicks() {
        return carouselEncoder.getRaw();
    }

    public void resetEncoder() {
        carouselEncoder.reset();
    }
}