/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Shooter;

public class ShooterTuner extends CommandBase {
  /**
   * Creates a new ShooterTuner.
   */

  Shooter shooter;
  Carousel carousel;
  CarouselCommand cc;
  int startTicks;


  public ShooterTuner(Shooter shooter, Carousel carousel, CarouselCommand cc) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.cc = cc;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 0.00001);
    shooter.vision.setMode("goalfinder");
    SmartDashboard.putNumber("Distance", SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[3]);
    System.out.println("Shooter Tuner going!");
    shooter.shoot();
    shooter.setHood(SmartDashboard.getNumber("Target Hood Angle", 60));
    shooter.setShooterRPM(SmartDashboard.getNumber("TSR", -1000));
    shooter.setTurret(SmartDashboard.getNumber("Turret Angle", 72));
    startTicks = carousel.getTicks();
    cc.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getSpeed() < SmartDashboard.getNumber("TSR", -1000) + 1000) {
      carousel.spin(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.stopAll();
    cc.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startTicks - 62805 >= carousel.getTicks();

  }
}
