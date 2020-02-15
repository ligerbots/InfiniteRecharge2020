/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Winch extends SubsystemBase {
    CANSparkMax winch;
    
    public Winch() {
        winch = new CANSparkMax(Constants.WINCH_MOTOR_ID, MotorType.kBrushless);
        // TODO: Need to add ratchet servo
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void deploy() {

    }

    public void climb(double speed) {
        winch.set(Constants.WINCH_SPEED);
    }
}