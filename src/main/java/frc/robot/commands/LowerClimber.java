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

public class LowerClimber extends CommandBase {
  /**
   * Creates a new ClimberCommand2.
   */
  Climber climber;

  enum ClimbingPhase {
    LOWER_WINCH, AUTO_LEVEL
  }

  ClimbingPhase currentPhase;

  public LowerClimber(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPhase = ClimbingPhase.LOWER_WINCH;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentPhase) {
      case LOWER_WINCH:
          climber.moveWinch(Constants.WINCH_CLIMB_HEIGHT - 300);
          if (Math.abs(climber.getWinchPosition() - (Constants.WINCH_CLIMB_HEIGHT - 300)) < 10) {
            currentPhase = ClimbingPhase.AUTO_LEVEL;
          }
        case AUTO_LEVEL:
          climber.autoLevel(true);
          break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPhase == ClimbingPhase.AUTO_LEVEL;
  }
}
