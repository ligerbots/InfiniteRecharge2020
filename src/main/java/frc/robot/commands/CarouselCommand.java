/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class CarouselCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  long lastTimeCheck;
  long timeCheck;
  boolean backwards;

  long lastBackTime;

  int currentTicks;
  int lastCheckpoint;
  int currentCheckpoint;

  final int fifthRotationTicks = 12561;
  final double pauseTime = 0.25; // seconds

  public CarouselCommand(Carousel carousel) {
    this.carousel = carousel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeCheck = System.nanoTime();
    lastTimeCheck = timeCheck;
    currentTicks = 0;
    lastCheckpoint = 0;
    currentCheckpoint = 0;
    backwards = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Output current", carousel.getCurrent());
    System.out.println("Output current: " + carousel.getCurrent());
    currentTicks = -carousel.getTicks();
    System.out.println(currentTicks);
    currentCheckpoint = currentTicks / fifthRotationTicks;
    if (carousel.getCurrent() > 6) {
      lastBackTime = System.nanoTime();
      backwards = true;
    }
    if (!backwards) {
      if (currentCheckpoint > lastCheckpoint) {
        lastCheckpoint = currentCheckpoint;
        lastTimeCheck = System.nanoTime();
      }
      if ((double)(System.nanoTime() - lastTimeCheck) / 1_000_000_000.0 > pauseTime) {
        carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
      }
      else {
        carousel.spin(0);
      }
    }
    else {
      carousel.spin(-0.6);
      if ((double)(System.nanoTime() - lastBackTime) / 1_000_000_000.0 > 0.3) {
        backwards = false;
      }
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
