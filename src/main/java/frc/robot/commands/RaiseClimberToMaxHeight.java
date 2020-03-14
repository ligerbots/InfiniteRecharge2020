/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RaiseClimberToMaxHeight extends CommandBase {
  /**
   * Creates a new ClimberCommand.
   */
  Climber climber;
  double winchHeight;
  double shoulderPosition;

  public RaiseClimberToMaxHeight(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderPosition = climber.getShoulderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
	// First we need to check the current height of the shoulder and the winch.
	winchHeight = climber.getWinchPosition();
	shoulderPosition = climber.getShoulderPosition();
	
	// We're trying to go up as fast as possible.
	// We have to limit the shoulder height until the winch gets high enough
	// so that we don't exceed frame perimeter
	if (winchHeight < Constants.WINCH_HEIGHT_FOR_LEVEL_BAR_AT_FRAME_PERIMETER) {
		// Winch isn't high enough. We need to limit the height of the shoulder
		if (shoulderPosition >= Constants.SHOULDER_HEIGHT_FOR_FRAME_PERIMETER) {
			// Need to hold the shoulder here
			climber.setShoulderIdleMode(IdleMode.kBrake);
			climber.setShoulderVoltage(0.0);
		} else {
			// Move the shoulder up
			climber.setShoulderVoltage(Constants.SHOULDER_SPEED_UP);
		}
		// Either way, the winch still has to go up.
		climber.setWinchVoltage(Constants.WINCH_SPEED_FAST);
	} else {
		// The winch is now past the frame perimeter.
		// Move the winch up and the shoulder up at max speed until they get to 
		// their max height.
		// Let's do the winch first
		if (winchHeight < Constants.WINCH_MAX_HEIGHT_TICK_COUNT) {
			climber.setWinchVoltage(Constants.WINCH_SPEED_FAST);
		} else {
			// Hold the winch at max height
			climber.stopWinch();
		}
		// Now we'll do the shoulder
		if (shoulderPosition < Constants.SHOULDER_HEIGHT_FOR_MAX_CLIMB) {
			// Raise the shoulder
			climber.setShoulderVoltage(Constants.SHOULDER_SPEED_UP);
		} else {
			// hold the winch at max height
			climber.setShoulderIdleMode(IdleMode.kBrake);
			climber.setShoulderVoltage(0.0);
		}
	}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	  // Hold both the winch and shoulder where they are
	  climber.stopWinch();
	  climber.setShoulderIdleMode(IdleMode.kBrake);
	  climber.setShoulderVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (winchHeight >= Constants.WINCH_MAX_HEIGHT_TICK_COUNT &&
			shoulderPosition >= Constants.SHOULDER_HEIGHT_FOR_MAX_CLIMB);
  }
}
