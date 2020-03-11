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
  double requestedWinchHeight;

  public WinchCommand(Climber climber, double winchHeight) {
    this.climber = climber;
    requestedWinchHeight = winchHeight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      climber.moveWinch(requestedWinchHeight);
      climber.setShoulderHeight(Constants.SHOULDER_CLIMB_HEIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getWinchPosition() >= requestedWinchHeight - 5;
  }

}
