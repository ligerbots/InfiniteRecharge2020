package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    private Relay spike;

    public Vision() {
        spike = new Relay(0);
    }

    @Override
    public void periodic() {
    }

    // public double[] getTargetInfo() {
    //     if (RobotBase.isSimulation()) {
    //         //TODO implement
    //     } else {
    //         visionInfo = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
            
    //     }        
    //     return visionInfo;
    // }

    // set vision processing mode, and set LED to match what is needed
    public void setMode(String mode) {
        setMode(mode, mode.equals("goalfinder"));
    }

    // set vision processing mode, and full control of LED
    // should be used only for special cases
    public void setMode(String mode, boolean led) {
        SmartDashboard.putString("vision/active_mode/selected", mode);
        setLedRing(led);
    }
    
    public boolean getStatus() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        // 1 = success, but it comes as a float, so test with a greater-than
        return visionData[1] > 0.1;
    }
    
    public double getDistance() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return visionData[3];
    }
    
    public double getRobotAngle() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return visionData[4] * 180.0 / Math.PI;
    }

    private void setLedRing (boolean on) {
        spike.set(on ? Value.kForward : Value.kReverse);
    }
}
