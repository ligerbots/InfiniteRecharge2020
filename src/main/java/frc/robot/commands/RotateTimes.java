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

public class RotateTimes extends CommandBase {
  /**
   * Creates a new RotateTimes.
   */

  Intake intake;
  int bufferCounter;
  int rotationCount;
  int rotationsToGo;
  OutColor startColor;

  public RotateTimes(Intake intake, int rotations) {
    rotationCount = 0;
    this.intake = intake;
    this.rotationsToGo = rotations;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startColor = intake.read();
    bufferCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationCount == rotationsToGo;
  }
}
