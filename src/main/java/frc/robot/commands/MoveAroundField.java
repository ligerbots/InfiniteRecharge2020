/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldMap;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

public class MoveAroundField extends CommandBase implements AutoCommandInterface {
  /**
   * Creates a new MoveAroundField.
   */

  // We need a Pose2d type to feed to the drive train
  private Pose2d robotPose;
  // These are for the values to be read from the Smart Dashboard
  private Integer start = 10;
  private Integer ball = 0;
  
  public MoveAroundField() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("start", 10);
    SmartDashboard.putNumber("ball", 0);
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
    return false;
  }

  // Allows the system to get the initial pose of this command
  public Pose2d getInitialPose() {
    start = (int)SmartDashboard.getNumber("start", 10);
    ball = (int)SmartDashboard.getNumber("ball", 0);

    // Use either start of ball to set robot pose.
    // 10 is the dummy default value for start.
    if (start < FieldMap.startPosition.length) {
      // The start value is valid. Use it to position the robot.
      robotPose = FieldMap.startPosition[start];
    } else if (ball < FieldMap.ballPosition.length) {
      // start value is invalid, so use the ball position with 0 rotation angle
      robotPose = new Pose2d(FieldMap.ballPosition[ball], new Rotation2d(0.0));
    } else {
      robotPose = new Pose2d();
    }
      return robotPose;
  }
}
