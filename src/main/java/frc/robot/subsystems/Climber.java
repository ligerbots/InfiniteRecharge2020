/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Climber extends SubsystemBase {
    private final CANSparkMax shoulder, winch; // declare new motors
    private DutyCycleEncoder throughBore;

    public Climber() {
        throughBore = new DutyCycleEncoder(9);
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); // init motor type and can id
        shoulder.setIdleMode(IdleMode.kBrake);
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); // init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
    }

    public void deploy() {

    }

    public void moveShoulder(final double speed) {
        shoulder.set(speed);
    }

    public void moveWinch(final double speed) {
        winch.set(speed); // set speed of motor
    }
}