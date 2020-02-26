/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public final CANSparkMax shoulder; // declare new motor
    CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    CANEncoder winchEncoder;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    public Climber() {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kBrake);
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winchEncoder = new CANEncoder(winch);
        winch.setIdleMode(IdleMode.kBrake);
        shoulderEncoder = new DutyCycleEncoder(9);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Location", shoulderEncoder.get());
    }

    public void deploy() {

    }

    public void moveShoulder(final double speed) {
        shoulder.set(speed); // set speed of motor 
    }

    public void moveWinch(final double speed) {
        winch.set(speed); // set speed of motor
    }

    public double getTicks(){
        winchEncoder.setPositionConversionFactor(Constants.WINCH_CONVERSION_FACTOR);
        return winchEncoder.getPosition();
    }

   
}