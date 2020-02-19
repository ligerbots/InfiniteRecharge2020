
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class ClimbCommand extends CommandBase {
    /**
     * Creates a new ClimbCommand.
     */
    DriveTrain robotDrive;
    Climber shoulder;

    public ClimbCommand(DriveTrain robotDrive, Climber shoulder) {
        this.robotDrive = robotDrive;
        this.shoulder = shoulder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // check if the back of the robot is pointing down
        // if so lower the shoulder to raise the robot
        if (robotDrive.getPitch() > 0)
            shoulder.moveShoulder(Constants.CLIMB_SPEED);
        else {
            // robot is high enough, stop the shoulder
            shoulder.moveShoulder(0);
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