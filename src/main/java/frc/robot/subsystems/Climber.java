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
    Double angle;
    public final CANSparkMax shoulder; // declare new motor
    public final CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    double zeroAngle = 0.44*360;
    double motorSpeed = 0.2; // set shoulder movement speed
    boolean deployed = false;
  
    CANEncoder winchEncoder;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    public Climber() {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kCoast); // set to Coast at startup
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
        angle = shoulderEncoder.get();
        winchEncoder = new CANEncoder(winch);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Location", shoulderEncoder.get());
        SmartDashboard.putNumber("Winch Encoder Location", winchEncoder.getPosition());
        //winch.set(winchSpeed = SmartDashboard.getNumber("winch speed", 0));
        //shoulder.set(SmartDashboard.getNumber("shoulder speed", 0));
    }

    public void deploy() {

    }

    public void moveShoulder(final double angle) {
        if (deployed) {
            while(true){
                if((shoulderEncoder.get()*360)-zeroAngle > angle+1) { // we cant go over so move motor back down
                    shoulder.set(-motorSpeed/2.0); // set speed of motor going down
                }else if ((shoulderEncoder.get()*360)-zeroAngle < angle-3){ // we can go under by 3 degrees
                        shoulder.set(motorSpeed); // set speed of motor going up
                } else {
                        shoulder.set(0); // dont go any where if the angle is ok
                        break; // stop looping
                }
            }
        }
    }

    //TODO: FIX THIS MESS!!! WE DON'T USE WHILE LOOPS BECAUSE WE HAVE EXECUTE
    public void deployShoulder() { 
        while((shoulderEncoder.get()*360) < (shoulderEncoder.get()*360)+5){ // we can go under by 3 degrees
                shoulder.set(motorSpeed); // set speed of motor going up
        }
        while((shoulderEncoder.get()*360)-zeroAngle > 3){ // we dont want to go under 0 degrees
            shoulder.set(-motorSpeed/2); // set speed of motor going up
        }
        shoulder.set(0); // dont go any where if the angle is ok
        angle = 0.0; // set shoulder angle to zero 
        deployed = true; // we are now deployed so allow controll
    }
    public void moveWinch(double winchSpeed) {
        winch.set(winchSpeed); // set speed of motor
    }

    public double getTicks(){
        winchEncoder.setPositionConversionFactor(Constants.WINCH_CONVERSION_FACTOR);
        return winchEncoder.getPosition();
    }

   public void resetWinchEncoder(){
       winchEncoder.setPosition(0);
   }
}