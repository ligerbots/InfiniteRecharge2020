package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;

// This interface will allow autonomous commands to provide the associated initial Pose2d

public interface AutoCommandInterface {
    public Pose2d getInitialPose();
}