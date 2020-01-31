/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  double[] visionInfo;
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0}; // TODO: Will prob have more values

  Shooter shooter;

  boolean startFlag;
  boolean started;

  public ShooterCommand(Shooter shooter) {
    this.shooter = shooter;

    startFlag = false;
  }

  public void warmUp () { // this function will start up the entire shooting sequence
    startFlag = true;
  }

  public void stop () { // this function will stop the entire shooting sequence
    startFlag = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionInfo = SmartDashboard.getNumberArray("visionInfo", empty); // TODO: need actual vision info
    if (startFlag) {
      shooter.warmUp();
      started = true;
    }
    else {
      started = false;
    }

    if (started) {
      if (visionInfo[0] != 0) { // figure out if we see a vision target
        shooter.prepareShooter(visionInfo[1]); // figure out which values is actually the distance
      }

      if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) && shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
        shooter.shoot();
      } //The allowed error here matters a lot
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
