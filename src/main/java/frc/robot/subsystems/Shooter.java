/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3;
    CANEncoder shooterEncoder;
    CANSparkMax flup;
    PWM hoodServo;
    double shootSpeed;

    // TODO: Need to add velocity PID for shooter

    public Shooter() {

        motor1 = new CANSparkMax(Constants.SHOOTER_ONE_CAN_ID,MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.SHOOTER_TWO_CAN_ID,MotorType.kBrushless);
        motor3 = new CANSparkMax(Constants.SHOOTER_THREE_CAN_ID,MotorType.kBrushless);
        shooterEncoder = new CANEncoder(motor1);
        hoodServo = new PWM(Constants.SHOOTER_SERVO_PWM_ID);
        flup = new CANSparkMax(Constants.SHOOTER_FLUP_CAN_ID,MotorType.kBrushless);
        motor2.follow(motor1);  //  We want motor1 to be master and motor2 and 3 follow the speed of motor1
        motor3.follow(motor1);  //  ^^^
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run every 20ms
    }

    public void shoot(double distance) {
        // TODO: The idea was that this would set the shooter speed
        // and hoodServo value based on the input distance.
    }

}