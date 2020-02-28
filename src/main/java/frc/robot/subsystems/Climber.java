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
    double currentAngle;
    double requestedAngle = Constants.SHOULDER_START_HEIGHT;
    double lastAngle;
    public final CANSparkMax shoulder; // declare new motor
    public final CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    double zeroAngle = 0.44*360;
    double shoulderSpeedUp = 0.8; // set shoulder movement speed
    double shoulderSpeedHold = 0.1; //This is not enough to lift the intake, but wll hold it in place
    double shoulderRateDown = 30.0/360.0; // a little under 2 seconds to get from max height to min height
    boolean deployed = false;
    boolean movingDown = false;
    boolean autoLevel = false;
  
    CANEncoder winchEncoder;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    private DriveTrain driveTrain;

    public Climber(DriveTrain driveTrain) {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kCoast); // set to Coast at startup
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
        currentAngle = shoulderEncoder.get();
        winchEncoder = new CANEncoder(winch);
        this.driveTrain = driveTrain;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Location", shoulderEncoder.get());
        SmartDashboard.putNumber("Winch Encoder Location", winchEncoder.getPosition());
        //winch.set(winchSpeed = SmartDashboard.getNumber("winch speed", 0));
        //shoulder.set(SmartDashboard.getNumber("shoulder speed", 0));

        // This should keep the shoulder at the requested height or let it down slowly
        currentAngle = shoulderEncoder.get();

        if (!autoLevel) {
            if (!movingDown) {
                // move up
                if (currentAngle > Constants.SHOULDER_MAX_HEIGHT) {
                    // Hold Position
                    shoulder.setVoltage(shoulderSpeedHold);
                }
                else {
                    if (currentAngle < requestedAngle) {
                        // Need to move up
                        shoulder.setVoltage(shoulderSpeedUp);
                    }
                    else {
                        // hold postion
                        shoulder.setVoltage(shoulderSpeedHold);
                    }
                }
            }
            else {
                // We're moving down. Check to see if we need to move slowly
                if (currentAngle < Constants.SHOULDER_MIN_VELOCITY_HEIGHT) {
                    // Let it coast down the last 10 degrees
                    shoulder.setIdleMode(IdleMode.kCoast);
                    shoulder.setVoltage(0.0);
                }
                else {
                    if (currentAngle < requestedAngle) {
                        // We've gone down far enough
                        shoulder.setVoltage(shoulderSpeedHold);
                        shoulder.setIdleMode(IdleMode.kBrake);
                    }
                    else {
                        // Now we get clever. We have to only allow it to fall at the
                        // shoulderRateDown
                        if ( lastAngle - currentAngle > shoulderRateDown * 0.02) {
                            // It's going too fast so we need to slow it down
                            shoulder.setVoltage(shoulderSpeedHold);
                        }
                        else {
                            // Let it coast for a while
                            shoulder.setIdleMode(IdleMode.kCoast);
                            shoulder.setVoltage(0.0);
                        }
                    }
                }
            }
        }
        // Auto levelling
        else {
            if (driveTrain.getPitch() < Constants.ROBOT_PITCH_ANGLE_FOR_CLIMB ) {
                // Need to lift the front
                shoulder.setVoltage(shoulderSpeedUp);
            }
            else {
                // Hold where it is
                shoulder.setIdleMode(IdleMode.kBrake);
                shoulder.setVoltage(0.0);
            }
        }
        // save the angle for next time
        lastAngle = currentAngle;
    }

    public void deploy() {
        // This is for the start of the match
        // We need to set arm to brake mode, raise it slowly to max height and then lower to min height
    }

    public void moveShoulder(final double angle) {
        
        requestedAngle = angle;

        // Limit max requested height
        if (requestedAngle > Constants.SHOULDER_MAX_HEIGHT) {
            requestedAngle = Constants.SHOULDER_MAX_HEIGHT;
        }

        // Limit min requested heght
        if (requestedAngle < Constants.SHOULDER_MIN_HEIGHT) {
            requestedAngle = Constants.SHOULDER_MIN_HEIGHT;
        }

        if (requestedAngle > currentAngle) {
            movingDown = false;
        }
        else {
            movingDown = true;
        }
        SmartDashboard.putBoolean("Moving Down", movingDown);
        SmartDashboard.putNumber("Shoulder Requested Angle", requestedAngle);
    }

        // Moving the shoulder to the correct angle will be done in the periodic() method


    //TODO: FIX THIS MESS!!! WE DON'T USE WHILE LOOPS BECAUSE WE HAVE EXECUTE
    public void deployShoulder() { 

        // raise shoulder to max height and then let it go down
        shoulder.set(0.1);
        while((shoulderEncoder.get()*360) < (shoulderEncoder.get()*360)+5){ // we can go under by 3 degrees
                shoulder.set(shoulderSpeedUp); // set speed of motor going up
        }
        while((shoulderEncoder.get()*360)-zeroAngle > 3){ // we dont want to go under 0 degrees
            shoulder.set(-shoulderSpeedUp/2); // set speed of motor going up
        }
        shoulder.set(0); // dont go any where if the angle is ok
        requestedAngle = 0.0; // set shoulder angle to zero 
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

   public double getShoulderPosition(){
       return shoulderEncoder.get();
   }

   public boolean atMinHeight(){
       return shoulderEncoder.get() < Constants.SHOULDER_MIN_VELOCITY_HEIGHT;
   }
}