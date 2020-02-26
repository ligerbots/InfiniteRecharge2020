/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
    DoubleSupplier angle;
    public final CANSparkMax shoulder; // declare new motor
    CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    double zeroAngle = shoulderEncoder.get()-Constants.MAX_SHOULDER_ANGLE;
    double motorSpeed = 0.2; // set shoulder movement speed
    double winchSpeed = 0.7; // set the wich speed when running

    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    public Climber(DoubleSupplier angle) {
        this.angle = angle;
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kBrake); // set to break when the motor is speed 0
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Location", shoulderEncoder.get());
        winch.set(winchSpeed = SmartDashboard.getNumber("winch speed", 0));
        shoulder.set(SmartDashboard.getNumber("shoulder speed", 0));
    }

    public void deploy() {

    }

    public void moveShoulder(final double angle) {
        if(shoulderEncoder.get() > angle) { // we cant go over so move motor back down
        shoulder.set(-motorSpeed); // set speed of motor going down
        }else if (shoulderEncoder.get() < angle-3){ // we can go under by 3 degrees
            shoulder.set(motorSpeed); // set speed of motor going up
        } else {
            shoulder.set(0); // dont go any where if the angle is ok
        }
    }
    public void moveShoulder(DoubleSupplier angle) {
        if(shoulderEncoder.get() > angle.getAsDouble()) { // we cant go over so move motor back down
            shoulder.set(-motorSpeed); // set speed of motor going down
        }else if (shoulderEncoder.get() < angle.getAsDouble()-3){ // we can go under by 3 degrees
                shoulder.set(motorSpeed); // set speed of motor going up
        } else {
                shoulder.set(0); // dont go any where if the angle is ok
        }
    }
    public void moveWinch() {
        winch.set(winchSpeed); // set speed of motor
    }
}