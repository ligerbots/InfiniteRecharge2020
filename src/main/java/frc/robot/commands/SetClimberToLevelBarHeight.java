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

public class SetClimberToLevelBarHeight extends CommandBase {
  /**
   * Creates a new ClimberCommand.
   */
  Climber climber;
  double winchHeight;
  double shoulderPosition;

  public SetClimberToLevelBarHeight(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	  // This command should only be called after the winch has reached max height
	  // So we only need to start the winch going and then stop the command when it
	  // gets tot he right spot
      climber.setWinchVoltage(Constants.WINCH_SPEED_SLOW);
	  // Make sure the shoulder stays about where it is
	  climber.setShoulderIdleMode(IdleMode.kBrake);
	  climber.setShoulderVoltage(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing to do here. The winch will keep going until we stop it.
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
    return (climber.getWinchPosition() >= Constants.WINCH_LEVEL_BAR_TICK_COUNT);
  }
}
