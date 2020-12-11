package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FieldMap;

public class Vision extends SubsystemBase {
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
        Translation2d goalDiff = FieldMap.goalCenterPoint.minus(driveTrain.getPose().getTranslation());
        double distance = goalDiff.getNorm();
        targetInfoSim[3] = distance / Constants.inchToMetersConversionFactor;

        double angleRobotGoal = Math.atan2(goalDiff.getY(), goalDiff.getX());
        targetInfoSim[4] = Math.toRadians(driveTrain.getHeading()) - angleRobotGoal;
        SmartDashboard.putNumberArray("vision/target_info", targetInfoSim);
    }

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
        return Math.toDegrees(visionData[4]);
    }

    public void setLedRing (boolean on) {
        spike.set(on ? Value.kForward : Value.kReverse);
    }
}
