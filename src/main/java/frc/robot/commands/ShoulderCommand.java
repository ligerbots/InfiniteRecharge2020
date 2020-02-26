/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShoulderCommand extends CommandBase {
    public final CANSparkMax shoulder; // declare new motor
    public final CANSparkMax winch; // declare new motor
    public final DutyCycleEncoder shoulderEncoder;
    double zeroAngle = 0.44*360.0;
    double motorSpeed = 0.2; // todo set shoulder movement speed
    double winchSpeed = 0.7; //todo  set the wich speed when running
    public Double angle;


    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    public ShoulderCommand() {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kBrake); // set to break when the motor is speed 0
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
        angle = shoulderEncoder.get();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((shoulderEncoder.get()*360)-zeroAngle > angle) { // we cant go over so move motor back down
        shoulder.set(-motorSpeed/2.0); // set speed of motor going down
    }else if ((shoulderEncoder.get()*360)-zeroAngle < angle-3){ // we can go under by 3 degrees
            shoulder.set(motorSpeed); // set speed of motor going up
    } else {
            shoulder.set(0); // dont go any where if the angle is ok
    }
  }
  public double getAngle() {
      return (shoulderEncoder.get()*360)-zeroAngle;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}