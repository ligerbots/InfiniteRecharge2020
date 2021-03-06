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
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {



    WPI_TalonSRX spinner;
    Encoder carouselEncoder;
    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    int ballCount = 0;


    public Carousel() {
        spinner = new WPI_TalonSRX(Constants.CAROUSEL_CAN_ID);
        spinner.setNeutralMode(NeutralMode.Brake);
        carouselEncoder = new Encoder(7, 8);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void spin(double speed) {
        spinner.set(ControlMode.PercentOutput, speed);
    }

    public double getCurrent () {
        return spinner.getStatorCurrent();
    }

    public void warmUp() {
        this.spin(Constants.CAROUSEL_SHOOTER_SPEED);
    }


    public int getTicks() {
        return carouselEncoder.getRaw();
    }

    public void resetEncoder () {
        carouselEncoder.reset();
    }

    public boolean isBallInFront () {
        return colorSensor.getProximity() > 110;
    }

    public int getColorSensorProximity() {
        return colorSensor.getProximity();
    }

    public void setBallCount (int ballCount) {
        this.ballCount = ballCount;
    }

    public void resetBallCount () {
        ballCount = 0;
    }

    public int getBallCount () {
        return ballCount;
    }
}