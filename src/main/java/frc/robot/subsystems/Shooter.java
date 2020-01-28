/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax motor1, motor2, motor3;
    PWM hoodServo;
    double shootSpeed;

    // TODO: Need to add velocity PID for shooter

    public Shooter() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void shoot(double distance) {
        // TODO: The idea was that this would set the shooter speed
        // and hoodServo value based on the input distance.
    }

}