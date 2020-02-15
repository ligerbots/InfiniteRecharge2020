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

public class Climber extends SubsystemBase {
    CANSparkMax shoulder;
    CANSparkMax winch;
    public Climber() {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        winch = new CANSparkMax(Constants.WINCH_MOTOR_ID, MotorType.kBrushless);
        // TODO: Need to add ratchet servo
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void deploy() {

    }

    public void moveShoulder(final double speed) {
        shoulder.set(speed);
    }

    public void wincher(final double speed) {
        winch.set(speed);
    }
}