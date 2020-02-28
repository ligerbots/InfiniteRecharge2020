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
import frc.robot.subsystems.Shooter;

public class FaceShootingTarget extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  double initialAngleOffset;
  double startingAngle;
  double acceptableError;
  DriveTrain robotDrive;
  DriveCommand driveCommand;
  Shooter shooter;

  public FaceShootingTarget(DriveTrain robotDrive, double acceptableError, DriveCommand driveCommand, Shooter shooter) {
    this.robotDrive = robotDrive;
    this.acceptableError = acceptableError;
    this.driveCommand = driveCommand;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveCommand.cancel();
    shooter.setLEDRing(true);
    SmartDashboard.putString("vision/active_mode/selected", "goalfinder");
    startingAngle = robotDrive.getHeading();
    initialAngleOffset = SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[4] * 180 / 3.1416;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(initialAngleOffset);
    robotDrive.allDrive(0, robotDrive.turnSpeedCalc(robotDrive.getHeading() - (startingAngle + initialAngleOffset)), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.allDrive(0, 0, false);
    driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(robotDrive.getHeading() - (startingAngle + initialAngleOffset)) < acceptableError || initialAngleOffset == 0.0;
  }
}
