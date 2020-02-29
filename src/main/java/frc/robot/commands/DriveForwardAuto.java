/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardAuto extends CommandBase {
  /**
   * Creates a new DriveForwardAuto.
   */
  DriveTrain robotDrive;
  long startTime;
  CarouselCommand carouselCommand;
  DriveCommand driveCommand;

  public DriveForwardAuto(DriveTrain robotDrive, CarouselCommand carouselCommand, DriveCommand driveCommand) {
    this.robotDrive = robotDrive;
    this.carouselCommand = carouselCommand;
    this.driveCommand = driveCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
    driveCommand.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotDrive.allDrive(0.5, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.allDrive(0, 0, false);
    carouselCommand.schedule();
    driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (double)(System.nanoTime() - startTime) / 1_000_000_000.0 > 0.65;
  }
}
