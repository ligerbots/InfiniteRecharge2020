/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {



    // TODO: Should this be part of the Shooter?
    WPI_TalonSRX spinner;

    public Carousel() {
        spinner = new WPI_TalonSRX(Constants.CAROUSEL_CAN_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void spin(double speed) {
        System.out.println(speed);
        spinner.set(ControlMode.PercentOutput, speed);
    }

    public void warmUp() {
        this.spin(Constants.CAROUSEL_SHOOTER_SPEED);
    }
}