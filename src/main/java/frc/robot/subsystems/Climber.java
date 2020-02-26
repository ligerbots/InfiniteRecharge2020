/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.ShoulderCommand;

public class Climber extends SubsystemBase {
    DoubleSupplier angle;
    public ShoulderCommand shoulder;
    boolean deployed = false;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    public Climber(DoubleSupplier angle) {
        this.angle = angle;
        shoulder = new ShoulderCommand();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Location", shoulder.shoulderEncoder.get());
        shoulder.winch.set(SmartDashboard.getNumber("winch speed", 0));
        shoulder.shoulder.set(SmartDashboard.getNumber("shoulder speed", 0));
        if(deployed) shoulder.angle = angle.getAsDouble(); // if we are deployed controll the arm

    }

    public void deploy() {

    }
    public void deployShoulder() { 
        shoulder.angle = (shoulder.getAngle()+5); // add 5* to lift shouder and un hook latch
        // need to wait
        shoulder.angle = 0.0; // set shoulder angle to zero 
        deployed = true; // we are now deployed so allow controll
    }
    public void deployWinch() {
        //winch.set(winchSpeed); // set speed of motor
    }
}