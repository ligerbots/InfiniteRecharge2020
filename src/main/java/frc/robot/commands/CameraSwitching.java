/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CameraSwitching extends CommandBase {
  /**
   * Creates a new CameraSwitching.
   */
  String cameraMode;
  public CameraSwitching(String mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    cameraMode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (cameraMode) {
      case "shooter" :
      SmartDashboard.putString("SmartDashboard/vision/selected", "shooter");
      break;
      case "intake" :
      SmartDashboard.putString("SmartDashboard/vision/selected", "intake");
      break;
      case "goalfinder" :
      SmartDashboard.putString("SmartDashboard/vision/selected", "goalfinder");
      break;
      case "ballfinder" :
      SmartDashboard.putString("SmartDashboard/vision/selected", "ballfinder");
      break;
      default:
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
    return false;
  }
}
