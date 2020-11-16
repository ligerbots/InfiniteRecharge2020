/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class FaceShootingTarget extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  double initialAngleOffset;
  double startingAngle;
  double acceptableError;
  double currentHeading;
  DriveTrain robotDrive;
  DriveCommand driveCommand;
  Shooter shooter;

  boolean oldOldCheck;
  boolean oldCheck;
  boolean check;

  boolean targetAcquired;

  long startTime;

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
    shooter.setTurret(79.5);
    targetAcquired = false;
    oldOldCheck = false;
    check = false;
    oldCheck = false;
    driveCommand.cancel();
    shooter.vision.setMode("goalfinder");
    startingAngle = robotDrive.getHeading();
    double distance = shooter.vision.getDistance();   
    initialAngleOffset = shooter.vision.getRobotAngle();
    startTime = System.nanoTime();
    System.out.println("Initial initial heading: " + robotDrive.getHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("starting angle", startingAngle);
    SmartDashboard.putNumber("initialAngleOffset", initialAngleOffset);

    if (targetAcquired) {
      currentHeading = robotDrive.getHeading();
      check = Math.abs(currentHeading - (startingAngle - initialAngleOffset)) < acceptableError && oldCheck;
      System.out.format("FaceShootingTarget: %3.2f%n", initialAngleOffset);
      robotDrive.allDrive(0, robotDrive.turnSpeedCalc(robotDrive.getHeading() - (startingAngle - initialAngleOffset)), false);

      oldCheck = Math.abs(currentHeading - (startingAngle - initialAngleOffset)) < acceptableError && oldOldCheck;

      oldOldCheck = Math.abs(currentHeading - (startingAngle - initialAngleOffset)) < acceptableError;
    }
    else {
      initialAngleOffset = shooter.vision.getRobotAngle();
      targetAcquired = initialAngleOffset != 0.0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FACE SHOOTING FINISHED");
    System.out.println("Current Heading: " + robotDrive.getHeading() + System.getProperty("line.separator") + 
    "Target Angle: " + (startingAngle - initialAngleOffset));
    robotDrive.allDrive(0, 0, false);
    Robot.angleErrorAfterTurn = currentHeading - (startingAngle - initialAngleOffset);
    driveCommand.schedule(); //TODO comment this back out
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(robotDrive.getHeading() - (startingAngle - initialAngleOffset)) < acceptableError && check) || (initialAngleOffset == 0.0 && (double)(System.nanoTime() - startTime) / 1_000_000_000.0 > 0.5);
  }
}
