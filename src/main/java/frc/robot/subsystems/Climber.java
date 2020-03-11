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
        SmartDashboard.putNumber("Shoulder Current", shoulder.getOutputCurrent());

        // We need to constantly check the current shoulder angle.
        currentShoulderAngle = shoulderEncoder.get();

        // If we're not auto-levelling
        if (!autoLevel) {

            // We're just going to use this periodic() method to make sure that the arm
            // doesn't go too high nor get driven too low.
            if (currentShoulderAngle > Constants.SHOULDER_MAX_HEIGHT_DONT_GO_ABOVE_EVER) {
                // Hold Position
                shoulder.setIdleMode(IdleMode.kBrake);
                shoulder.setVoltage(0.0);
            } else {
                // Check if we're going up or down
                if (shoulderMovingDown) {
                    // If we're low enough, just coast
                    if (currentShoulderAngle < Constants.SHOULDER_MIN_VELOCITY_HEIGHT) {
                        // Let it coast down the last few degrees
                        setShoulderIdleMode(IdleMode.kCoast);
                        setShoulderVoltage(0.0);
                    } else {
                        // We need to go down slowly
                        // We have to only allow it to fall at the shoulderRateDown
                        // The angle is only allowed to change by the rate * .02 because .02 is the FMS
                        // frame rate.
                        // And since we're going down, we dont need an absolute value.
                        if (lastShoulderAngle - currentShoulderAngle > Constants.SHOULDER_RATE_DOWN * 0.02) {
                            // It's going too fast so we need to slow it down
                            setShoulderIdleMode(IdleMode.kBrake);
                        } else {
                            // Let it coast for a while
                            setShoulderIdleMode(IdleMode.kCoast);
                            setShoulderVoltage(0.0);
                        }
                    }
                } else {
                    // We're moving up
                    // If we're not high enough, then set speed to go up
                    if (currentShoulderAngle < requestedShoulderAngle) {
                        shoulder.setVoltage(Constants.SHOULDER_SPEED_UP);
                    } else {
                        // We're still going up, but we're high enough
                        // Set shoulder speed to hold. If it falls, the previous
                        // if check will raise it a little
                        shoulder.setIdleMode(IdleMode.kBrake);
                        shoulder.setVoltage(0.0);
                    }
                }
            }
        } else {
            // Auto levelling
            // System.out.println(" " + driveTrain.getPitch());
            if (driveTrain.getPitch() > Constants.ROBOT_PITCH_ANGLE_FOR_CLIMB) {
                // Need to lift the front
                shoulder.setVoltage(Constants.SHOULDER_SPEED_LEVEL);
            } else {
                // Hold where it is
                shoulder.setIdleMode(IdleMode.kBrake);
                shoulder.setVoltage(0.0);
            }
        }
        // save the angle for next time
        lastShoulderAngle = currentShoulderAngle;
    }

    public void setShoulderHeight(double angle) {
        // This just sets parameters to be used in the periodic() method.
        // Moving the shoulder to the correct angle will be done in the periodic() method

        requestedShoulderAngle = angle;
        
        // Limit max requested height
        if (requestedShoulderAngle > Constants.SHOULDER_MAX_HEIGHT_FOR_FRAME_PERIMETER) {
            requestedShoulderAngle = Constants.SHOULDER_MAX_HEIGHT_FOR_FRAME_PERIMETER;
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

    public void setShoulderIdleMode(IdleMode idleMode) {
        shoulder.setIdleMode(idleMode);
    }

    public void setShoulderVoltage(double voltage) {
        shoulder.setVoltage(voltage);
    }
	
	public void setWinchVoltage(double voltage) {
		winch.setVoltage(voltage);
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