/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    CANSparkMax winch, shoulder;

    CANEncoder winchEncoder;
    // TODO: Need variable for the through bore encoder on the shoulder
    // TODO: May need a servo for deploying climber

    static final double WINCH_POWER = 0.25;

    public Climber() {
        winch = new CANSparkMax(Constants.WINCH_CAN_ID,MotorType.kBrushless);
        shoulder = new CANSparkMax(Constants.SHOULDER_CAN_ID,MotorType.kBrushless);

        winchEncoder = new CANEncoder(winch);
        // TODO: Need to add ratchet servo
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void deploy() {

    }

    public void driveWinch() {
        winch.set(WINCH_POWER);
    }

    public void stopWinch() {
        winch.set(0);
    }

}