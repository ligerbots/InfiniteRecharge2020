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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.CircularBuffer;

public class ShootOne extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 
  double waitTime;
  double startTime;

  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;

  boolean startShooting;

  CarouselCommand carouselCommand;
  DriveCommand driveCommand;

  int initialCarouselTicks;

  long stableRPMTime;
  boolean startedTimerFlag;
  boolean foundTarget;

  private CircularBuffer kFEstimator = new CircularBuffer(20);

  public enum ControlMethod {
    SPIN_UP, // PIDF to desired RPM
    HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
  }

  ControlMethod currentControlMode;
  boolean rescheduleDriveCommand;

  public ShootOne(Shooter shooter, Carousel carousel, DriveTrain robotDrive, double waitTime, CarouselCommand carouselCommand, DriveCommand driveCommand, boolean rescheduleDriveCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;
    this.waitTime = waitTime;
    this.carouselCommand = carouselCommand;
    this.driveCommand = driveCommand;
    this.rescheduleDriveCommand = rescheduleDriveCommand;
    startShooting = false;
  }

  public void rapidFire() {
    carousel.spin(0.5);
    shooter.shoot();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    foundTarget = false;
    shooter.calibratePID(0.000085, 0.000000035, 0, 6.776 * 0.00001);
    driveCommand.cancel();
    startTime = System.nanoTime();
    shooter.vision.setMode("goalfinder");
    //TODO: remember to set to shooting camera mode!!
    carouselCommand.cancel();
    currentControlMode = ControlMethod.SPIN_UP;

    // stor current carouselTick value
    initialCarouselTicks = carousel.getTicks();
     

    angleError = shooter.vision.getRobotAngle();
    distance = shooter.vision.getDistance();

    shooter.prepareShooter(distance);
    currentControlMode = ControlMethod.SPIN_UP;
    startedTimerFlag = false;
    //shooter.shoot();
    shooter.setTurretAdjusted(-angleError);
  }

  // Called every time the scheduler runs while the command is scheduled.
  double angleError;
  double distance;

  boolean speedOnTarget = false;
  boolean hoodOnTarget = false;
  boolean angleOnTarget = false;

  @Override
  public void execute() {

    if (distance != 0.0) {
      foundTarget = true;
    }

    if (!foundTarget) {

      distance = shooter.vision.getDistance();
    }

    angleError = shooter.vision.getRobotAngle();

    //System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance) + "   Current Speed: " + shooter.getSpeed() + " ");

    if (currentControlMode == ControlMethod.SPIN_UP){ 
      if (shooter.speedOnTarget(-shooter.calculateShooterSpeed(distance), 15)) {
        if (startedTimerFlag) {
          if (System.nanoTime() - stableRPMTime > 0.2 * 1_000_000_000) {
            currentControlMode = ControlMethod.HOLD;
          }
        }
        else {
          stableRPMTime = System.nanoTime();
          startedTimerFlag = true;
        }
      }
      else {
        startedTimerFlag = false;
      }
    }
    else if (currentControlMode == ControlMethod.HOLD) {
      //kFEstimator.addValue(val);
      shooter.calibratePID(0, 0, 0, 1 / (5700 * 2.6666));
    }

  
    speedOnTarget = (shooter.speedOnTarget(-shooter.calculateShooterSpeed(distance), 8) && currentControlMode == ControlMethod.HOLD) || (double)(System.nanoTime() - startTime) / 1_000_000_000 > 2.5; //TODO: May need to adjust acceptable error
    hoodOnTarget = (double)(System.nanoTime() - startTime) / 1_000_000_000 > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));

    if (speedOnTarget && hoodOnTarget)
        rapidFire();

  }

  // if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) && shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
  //   shooter.shoot();
  // } //The allowed error here matters a lot

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.vision.setLedRing(false);
    carousel.setBallCount(0);
    carouselCommand.schedule();
    if (rescheduleDriveCommand) {
      driveCommand.schedule();
    }
  }

  /*public double estimateKF (double rpm, double voltage) {
    final double speed_in_ticks_per_20ms = 
  }*/

  // Returns true when the command should end.



  @Override
  public boolean isFinished() {
    // TODO: this should just check to see if the carousel has rotated 5 CAROUSEL_FIFTH_ROTATION_TICKS intervals
    return (carousel.getTicks() - initialCarouselTicks) < -1 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS || (distance == 0.0 && (double)(System.nanoTime() - startTime) / 1_000_000_000.0 > 2.0);
            /*((double)System.nanoTime() - startTime) / 1_000_000_000.0 > 7.0;*/
    // if (waitTime == 0.0) {
    //   return false;
    // }
    // else {
    //   return ((System.nanoTime() - startTime) / 1_000_000_000.0 >= waitTime);
    // }
  }
}
