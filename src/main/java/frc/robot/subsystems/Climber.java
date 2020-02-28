/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    double currentShoulderAngle;
    double requestedShoulderAngle;
    double tempRequestedShoulderAngle;
    double lastShoulderAngle;
    public final CANSparkMax shoulder; // declare new motor
    public final CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    double shoulderSpeedUp = Constants.SHOULDER_SPEED_UP; // set shoulder movement speed
    double shoulderSpeedHold = Constants.SHOULDER_SPEED_HOLD; //This is not enough to lift the intake, but wll hold it in place
    double shoulderRateDown = Constants.SHOULDER_RATE_DOWN; // a little under 2 seconds to get from max height to min height
    boolean deployed = false;
    boolean shoulderMovingDown = false;
    boolean autoLevel = false;
    boolean hookGoingUp = true;
    double currentWinchHeight;
    double requestedWinchHeight;
  
    CANEncoder winchEncoder;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    private DriveTrain driveTrain;

    public Climber(DriveTrain driveTrain) {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kCoast); // set to Coast at startup
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
        currentShoulderAngle = shoulderEncoder.get();
        winchEncoder = new CANEncoder(winch);
        this.driveTrain = driveTrain;
    }

    @Override
    public void periodic() {

        // This periodic() method should be used to move the shoulder and the winch to the
        // requested setting.

        // Commands should use other methods to set parameters for the shoulder and winch
        // This method adjusts the shoulder and winch to get to the requested settings

        // SmartDashboard.putNumber("Shoulder Encoder Location", shoulderEncoder.get());
        // SmartDashboard.putNumber("Winch Encoder Location", winchEncoder.getPosition());
        //winch.set(winchSpeed = SmartDashboard.getNumber("winch speed", 0));


        // Let's do the shoulder first
        // This should keep the shoulder at the requested height or let it down slowly
        currentShoulderAngle = shoulderEncoder.get();

        if (!autoLevel) {
            if (!shoulderMovingDown) {
                // move up
                if (currentShoulderAngle > Constants.SHOULDER_MAX_HEIGHT) {
                    // Hold Position
                    shoulder.setVoltage(shoulderSpeedHold);
                }
                else {
                    if (currentShoulderAngle < requestedShoulderAngle) {
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
                if (currentShoulderAngle < Constants.SHOULDER_MIN_VELOCITY_HEIGHT) {
                    // Let it coast down the last 10 degrees
                    shoulder.setIdleMode(IdleMode.kCoast);
                    shoulder.setVoltage(0.0);
                }
                else {
                    if (currentShoulderAngle < requestedShoulderAngle) {
                        // We've gone down far enough
                        shoulder.setVoltage(shoulderSpeedHold);
                        shoulder.setIdleMode(IdleMode.kBrake);
                    }
                    else {
                        // Now we get clever. We have to only allow it to fall at the
                        // shoulderRateDown
                        if ( lastShoulderAngle - currentShoulderAngle > shoulderRateDown * 0.02) {
                            // It's going too fast so we need to slow it down
                            shoulder.setIdleMode(IdleMode.kBrake);
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
            System.out.println(" " + driveTrain.getPitch());
            if (driveTrain.getPitch() > Constants.ROBOT_PITCH_ANGLE_FOR_CLIMB ) {
                // Need to lift the front
                shoulder.setVoltage(Constants.SHOULDER_SPEED_LEVEL);
            }
            else {
                // Hold where it is
                shoulder.setIdleMode(IdleMode.kBrake);
                shoulder.setVoltage(0.0);
            }
        }
        // save the angle for next time
        lastShoulderAngle = currentShoulderAngle;

        // Now let's do the winch

        currentWinchHeight = winchEncoder.getPosition();

        if (requestedWinchHeight < currentWinchHeight) {
            // Just stop the winch
            stopWinch();
        }
        else {
            winch.setVoltage(Constants.WINCH_SPEED_FAST);
        }

        // The winch only moves in one direction
        // This will also stop the winch when it gets to the desired set point
        /*if (requestedWinchHeight < currentWinchHeight) {
            // Just stop the winch
            stopWinch();
        }
        else {
            // If we're we have to make sure that the winch
            // is below the frame perimeter height
            if (currentShoulderAngle >= Constants.SHOULDER_HEIGHT_FOR_FRAME_PERIMETER) {
                // if the winch is too low, we need to lower the shoulder to avoid exceeding
                // frame perimeter
                if (currentWinchHeight < Constants.WINCH_HEIGHT_FOR_LEVEL_BAR_AT_FRAME_PERIMETER) {
                    stopWinch();
                    tempRequestedShoulderAngle = requestedShoulderAngle;
                    requestedShoulderAngle = Constants.SHOULDER_HEIGHT_FOR_FRAME_PERIMETER;
                }
                else {
                    // winch is past the frame perimeter danger point, so we can raise the shoulder
                    requestedShoulderAngle = tempRequestedShoulderAngle;
                    // and let the winch go
                    winch.setVoltage(Constants.WINCH_SPEED_FAST);
                }
            } else {
                // Shoulder is low enough, just go
                // We want to go fast up to max height
                if (currentWinchHeight < Constants.WINCH_MAX_HEIGHT_TICK_COUNT) {
                    winch.setVoltage(Constants.WINCH_SPEED_FAST);
                }
                else {
                    // We're beyond max height. We're ether going to go slow to get to the
                    // level bar, or climb
                    if (currentWinchHeight < Constants.WINCH_HEIGHT_FOR_LEVEL_BAR_AT_FRAME_PERIMETER) {
                        // We're beyond max height so go slow until we get to the level bar height
                        winch.setVoltage(Constants.WINCH_SPEED_SLOW);
                    }
                    else {
                        // We're climbing
                        autoLevel = true;
                        winch.setVoltage(Constants.WINCH_SPEED_CLIMB);
                    }
                }
            }
        }*/
    }

    public void moveShoulder(final double angle) {
        // This just sets parameters to be used in the periodic() method.
        // Moving the shoulder to the correct angle will be done in the periodic() method

        requestedShoulderAngle = angle;

        // Limit max requested height
        if (requestedShoulderAngle > Constants.SHOULDER_MAX_HEIGHT) {
            requestedShoulderAngle = Constants.SHOULDER_MAX_HEIGHT;
        }

        // Limit min requested heght
        if (requestedShoulderAngle < Constants.SHOULDER_MIN_HEIGHT) {
            requestedShoulderAngle = Constants.SHOULDER_MIN_HEIGHT;
        }

        if (requestedShoulderAngle > currentShoulderAngle) {
            shoulderMovingDown = false;
        }
        else {
            shoulderMovingDown = true;
        }
        SmartDashboard.putBoolean("Moving Down", shoulderMovingDown);
        SmartDashboard.putNumber("Shoulder Requested Angle", requestedShoulderAngle);
    }


    public void moveWinch(double winchHeight) {
        requestedWinchHeight = winchHeight;
    }

    public void stopWinch() {
        winch.setVoltage(0.0);
        winch.setIdleMode(IdleMode.kBrake);
    }

    public void coastWinch() {
        winch.setIdleMode(IdleMode.kCoast);
    }

    public double getWinchPosition(){
        return winchEncoder.getPosition();
    }

   public void resetWinchEncoder(){
       winchEncoder.setPosition(0);
   }

   public double getShoulderPosition(){
       return shoulderEncoder.get();
   }

   public boolean shoulderAtMinHeight(){
       return shoulderEncoder.get() < Constants.SHOULDER_MIN_VELOCITY_HEIGHT;
   }

   public void autoLevel(boolean autoLevel){
       this.autoLevel = autoLevel;
   }

   public boolean autoLeveling(){
       return autoLevel;
   }

   public boolean shoulderOnTarget () {
       return Math.abs(requestedShoulderAngle - shoulderEncoder.get()) <= 0.05;
   }
}