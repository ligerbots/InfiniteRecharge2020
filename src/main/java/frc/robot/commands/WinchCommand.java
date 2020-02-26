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

public class WinchCommand extends CommandBase {
  Climber climber;
  double winchTicks;

  public WinchCommand(Climber climber) {
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      winchTicks = climber.getTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    winchUp();
    winchMax();
    winchDown();
    winchClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.moveWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void winchUp() {
    if (winchTicks < Constants.WINCH_LEVEL_BAR_TICK_COUNT_UP) {
      climber.moveWinch(Constants.WINCH_SPEED_FAST);
    }
    else {
      climber.moveWinch(0.0);
    }
  }

  public void winchMax() {
    if (winchTicks < Constants.WINCH_MAX_HEIGHT_TICK_COUNT && winchTicks > Constants.WINCH_LEVEL_BAR_TICK_COUNT_UP) {
      climber.moveWinch(Constants.WINCH_SPEED_SLOW);
    }
    else {
      climber.moveWinch(0.0);
    }
  }

  public void winchDown() {
    if (winchTicks > Constants.WINCH_MAX_HEIGHT_TICK_COUNT && winchTicks < Constants.WINCH_LEVEL_BAR_TICK_COUNT_DOWN) {
      climber.moveWinch(Constants.WINCH_SPEED_SLOW);
    }
    else {
      climber.moveWinch(0.0);
    }
  }

  public void winchClimb() {
    if (winchTicks > Constants.WINCH_LEVEL_BAR_TICK_COUNT_DOWN) {
      climber.moveWinch(Constants.WINCH_SPEED_CLIMB);
    }
  }
}
