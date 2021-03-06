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

public class VisionTargetDistance extends CommandBase {
  /**
   * Creates a new VisionTargetDistance.
   */
  Shooter shooter;
  public VisionTargetDistance(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("Getting distance");

      // This is only for testing
      // Turn on the LED but put the camera into "Intake", which does not find the target!!
      
      shooter.vision.setMode("intake");


      //SmartDashboard.putNumber("Distance", SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[3]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
