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
   /*
    public void setLed(boolean on) {
        shooter.setLEDRing(on); TODO fix this
    }
    */
    public void setMode(String mode) {
        SmartDashboard.putString("vision/active_mode/selected", mode);
    }
    
    public double getDistance() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0});
        return visionData[3];
    }
    
    public double getRobotAngle() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0});
        return visionData[4] * 180 / Math.PI;
    }
}