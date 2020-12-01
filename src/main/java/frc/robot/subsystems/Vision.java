package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldMap;

public class Vision extends SubsystemBase{
    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    private Relay spike;
    private DriveTrain driveTrain;
    private double[] targetInfoSim = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    public Vision(DriveTrain driveTrain) {
        spike = new Relay(0);
        this.driveTrain = driveTrain;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        Transform2d goalDif = driveTrain.getPose().minus(FieldMap.goalCenterPoint);
        Translation2d goalDifTranslation = goalDif.getTranslation();
        double distance = Math.sqrt(Math.pow(goalDifTranslation.getX(), 2) +  
        Math.pow(goalDifTranslation.getY(), 2));
        targetInfoSim[3] = distance;
        SmartDashboard.putNumberArray("vision/target_info", targetInfoSim);
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

    public void setLedRing (boolean on) {
        spike.set(on ? Value.kForward : Value.kReverse);
    }
}
