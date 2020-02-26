/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.OutColor;

public class ControlPannelRotate extends CommandBase {
  /**
   * Creates a new RotateTimes.
   */

  Intake intake;
  int rotationCount;
  int rotationsToGo;
  OutColor startColor;


  OutColor currentColor; // YJ: current color read by the color sensor
  OutColor oldColor;
  int oldColorTicks;

  boolean newColor;
  boolean startingFlag;

  public ControlPannelRotate(Intake intake, int rotations) {
    rotationCount = 0;
    this.intake = intake;
    this.rotationsToGo = rotations *  2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startColor = intake.read();
    oldColor = startColor;
    oldColorTicks = 0;
    intake.run(0.25); // YJ: Placeholder for the intake speed
    newColor = false;
    startingFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    oldColor = currentColor;
    currentColor = intake.read();
    if (oldColor == currentColor) {
      oldColorTicks += 1;
    }
    else {
      oldColorTicks = 0;
    }
    if (oldColorTicks >= 3) {
      newColor = false;
    }
    else {
      newColor = true; 
      startingFlag = false;
    }
    if (currentColor == startColor && !newColor && !startingFlag){ // if the newColor reaches the startColor once, it increases the rotationCount by 1/2
      rotationCount += 1; // There are two of each color opposite each other
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationCount == rotationsToGo;
  }
}
