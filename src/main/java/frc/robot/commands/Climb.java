/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  /**
   * Creates a new Climb Command.
   */
  Climber climber;


  public Climb(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	  // This command should only be called after the hook is on the bar.
	  // We're going to start the winch going and set the shoulder to auto-level
      climber.setWinchVoltage(Constants.WINCH_SPEED_CLIMB);
	  climber.autoLevel(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	  // Nothing to do here.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	  // Hold the winch but let the shoulder continue to auto level until the end of the match
	  climber.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getWinchPosition() > Constants.WINCH_CLIMB_HEIGHT;
  }
}
