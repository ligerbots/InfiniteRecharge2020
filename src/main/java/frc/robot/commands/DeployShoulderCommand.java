/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DeployShoulderCommand extends CommandBase {
  /**
   * Creates a new ShoulderCommand.
  */
 
  Climber climber;
  boolean deployed = false;
  boolean shoulderMovingDown = false;
  double currentShoulderAngle;
  double requestedShoulderAngle;
  double tempRequestedShoulderAngle;
  double lastShoulderAngle;
  
  public DeployShoulderCommand(Climber climber) {
      this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.shoulder.setIdleMode(IdleMode.kCoast);
    deployed = false;
    shoulderMovingDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (deployed) {
      // Just go down
      climber.setShoulderHeight(Constants.SHOULDER_MIN_HEIGHT);
    }
    else {
      // Need to go up to release hook
      climber.setShoulderHeight(Constants.SHOULDER_RELEASE_HEIGHT);
      if (climber.getShoulderPosition() >= Constants.SHOULDER_RELEASE_HEIGHT) {
        deployed = true;
        SmartDashboard.putBoolean("ShoulderDeployed", deployed);
      }
    }
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // We should already be here, but set it this way just in case.
    // We don't want the arm to drop free fall
    climber.setShoulderHeight(Constants.SHOULDER_MIN_HEIGHT);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // This is finished once we set the min height since the subsystem will take it the rest of the way
    return climber.shoulderAtMinHeight();
  }
}
