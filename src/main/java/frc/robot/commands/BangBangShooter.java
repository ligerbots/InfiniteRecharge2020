/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//
// PaulR Nov 2020: this is not going to work, and is not actually used!!
//

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionMode;

public class BangBangShooter extends CommandBase {
  /**
   * Creates a new BangBangShooter.
   */

  int initialCarouselTicks;

  double angleError;
  double distance;

  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;

  CarouselCommand carouselCommand;
  DriveCommand driveCommand;

  long stableRPMTime;
  boolean startedTimerFlag;
  double startTime;

  public enum ControlMethod {
    SPIN_UP, // PIDF to desired RPM
    HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
  }

  ControlMethod currentControlMode;


  public BangBangShooter(Shooter shooter, Carousel carousel, DriveTrain robotDrive, CarouselCommand carouselCommand, DriveCommand driveCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;
    this.carouselCommand = carouselCommand;
    this.driveCommand = driveCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void rapidFire() {
    carousel.spin(0.5);
    shooter.shoot();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
    driveCommand.cancel();
    shooter.vision.setMode(VisionMode.GOALFINDER);
    carouselCommand.cancel();

    // stor current carouselTick value
    initialCarouselTicks = carousel.getTicks();

    angleError = shooter.vision.getRobotAngle();
    distance = shooter.vision.getDistance();

    shooter.prepareShooter(distance);

    startedTimerFlag = false;

  }

  // Called every time the scheduler runs while the command is scheduled.

  boolean speedOnTarget;
  boolean hoodOnTarget;
  boolean angleOnTarget;

  @Override
  public void execute() {
    angleError = shooter.vision.getRobotAngle(); 

    System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance) + "   Current Speed: " + shooter.getSpeed() + "   " + currentControlMode);


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
      if (shooter.getSpeed() <= shooter.calculateShooterSpeed(distance)) {
        shooter.setShooterVoltage(12);
      }
      else {
        shooter.setShooterVoltage(0);
      }
    }


    if (shooter.vision.getStatus()) { // figure out if we see a vision target
  
      if (Math.abs(angleError) > 5) {
        robotDrive.allDrive(0, robotDrive.turnSpeedCalc(angleError), false);
      }
      else {
        robotDrive.allDrive(0, 0, false);
        //shooter.setTurret(angleError *  Math.signum(angleError));
      }

      speedOnTarget = shooter.speedOnTarget(-shooter.calculateShooterSpeed(distance), 15) && currentControlMode == ControlMethod.HOLD; //TODO: May need to adjust acceptable error
      hoodOnTarget = (double)(System.nanoTime() - startTime) / 1_000_000_000 > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));
      angleOnTarget = Math.abs(angleError) <= 4.5; // They should be opposites so I added them

      if (angleOnTarget) {
          shooter.setTurretAdjusted(-angleError);
      }

      if (speedOnTarget && hoodOnTarget && angleOnTarget)
          rapidFire();
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.vision.setMode(VisionMode.INTAKE);
    carousel.setBallCount(0);
    carouselCommand.schedule();
    driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (carousel.getTicks() -initialCarouselTicks) > 5 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
  }
}
