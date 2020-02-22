/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FaceShootingTarget extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  double angleOffset;
  double acceptableError;
  DriveTrain robotDrive;
  public FaceShootingTarget(DriveTrain robotDrive, double acceptableError) {
    this.robotDrive = robotDrive;
    this.acceptableError = acceptableError;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleOffset = SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[4];
    robotDrive.allDrive(0, robotDrive.turnSpeedCalc(angleOffset));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleOffset) < acceptableError;
  }
}
