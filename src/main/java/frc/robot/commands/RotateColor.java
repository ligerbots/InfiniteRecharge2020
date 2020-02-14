/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.OutColor;

public class RotateColor extends CommandBase {
  /**
   * Creates a new RotateColor.
   */

  Intake intake;
  OutColor targetColor;
  OutColor currentColor;

  public RotateColor(Intake intake, OutColor targetColor) {
    this.intake = intake;
    this.targetColor = targetColor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("SmartDashboard/vision/selected", "shooter");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentColor = intake.read();
    intake.run(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("SmartDashboard/vision/selected", "intake");
    return currentColor == targetColor;
  }
}
