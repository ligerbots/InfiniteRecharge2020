package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

// This interface will allow autonomous commands to provide the associated initial Pose2d

public interface AutoCommandInterface extends Command {
    public Pose2d getInitialPose();

    // This also needs to have all the methods of a Command.
    // The commands that implement this interface will all have implementations of
    // these methods
    public void initialize();

    public void execute();

    public void end(boolean isInterrupted);

    public boolean isFinished();

    // public void isInterrupted();
    default void schedule() {
        Command.super.schedule();
    };
}