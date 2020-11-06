package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase{
    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    private double[] visionInfo;

    public Vision() {
        visionInfo = EMPTY_TARGET_INFO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        
    }

    public double[] getTargetInfo() {
        if (RobotBase.isSimulation()) {
            //TODO implement
        } else {
            visionInfo = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
            
        }        
        return visionInfo;
    }
}