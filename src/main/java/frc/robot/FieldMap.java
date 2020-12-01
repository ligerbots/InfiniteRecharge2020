package frc.robot;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class FieldMap {

    // Field marks, for reference
    public static final double fieldWidth = 323  * Constants.inchToMetersConversionFactor;
    public static final double fieldLength = 629.25 * Constants.inchToMetersConversionFactor;
    // Distance to the center of the white tape line
    public static final double startLineX = fieldLength - (12 * 10 + 1) * Constants.inchToMetersConversionFactor;

    // Robot dimensions with bumpers
    public static final double robotWidth = 34.75 * Constants.inchToMetersConversionFactor;
    public static final double robotLength = 37.5 * Constants.inchToMetersConversionFactor;

    // half dimensions - for getting the center point of the robot
    public static final double rW2 = robotWidth/2.0;
    public static final double rL2 = robotLength/2.0;
    
    
    // centerline is the robot starting position
    public static final double targetCenterPointY = 94.66 * Constants.inchToMetersConversionFactor; // of target
    // the position of the Our goal
    public static final Translation2d goalCenterPoint = new Translation2d(fieldLength, targetCenterPointY);
    // center the robot on the startpoint (Inside of back frame rail lines up with near edge of line)
    public static final double startPositionX = startLineX + robotLength/2 - 2 * Constants.inchToMetersConversionFactor; 
    // center line through long axis of trench
    public static final double friendlyTrenchY = 27.75 * Constants.inchToMetersConversionFactor; 
    // X position of the two enemy trench balls closest to our goal
    public static final double enemyTrenchBallX = (10 * 12 + 258.9) * Constants.inchToMetersConversionFactor;
    // Ball in enemy trench close to our goal closest to wall
    public static final double enemyTrenchBall1Y = fieldWidth - (27.75 + 9.25) * Constants.inchToMetersConversionFactor;
    // Other Ball in enemy trench close to our goal
    public static final double enemyTrenchBall2Y = fieldWidth - (27.75 - 9.25) * Constants.inchToMetersConversionFactor;       
    
    // public static ArrayList<Translation2d> ballPositions = new ArrayList<Translation2d>();
    public static Translation2d[] ballPosition = new Translation2d[12];
    public static Pose2d[] startPosition = new Pose2d[5];
    // public static Translation2d[] controlPanel = new Translation2d[0];
    // public static Translation2d[] climberPoint = new Translation2d[2];

    public static ArrayList<Translation2d> wayPointsA = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsB = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsAlpha = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsBeta = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsGamma = new ArrayList<Translation2d>();

    static {

        // start positions are in terms of the robot center (x,y)

        // lined up with target center
        startPosition[0] = new Pose2d(startPositionX, targetCenterPointY, new Rotation2d(0.0));
        // 40 inches closer to center of field
        startPosition[1] = new Pose2d(startPositionX, targetCenterPointY + 40.0 * Constants.inchToMetersConversionFactor, new Rotation2d(0.0));
        // 40 inches closer to wall (relative to target center)
        startPosition[2] = new Pose2d(startPositionX, targetCenterPointY - 40.0 * Constants.inchToMetersConversionFactor, new Rotation2d(0.0));
        // Lined up with center line of trench
        startPosition[3] = new Pose2d(startPositionX, friendlyTrenchY, new Rotation2d(0.0));
         // centered in front of enemy feeder station
        startPosition[4] = new Pose2d(startPositionX, fieldWidth - 94.66 * Constants.inchToMetersConversionFactor, new Rotation2d(0.0));

        // locations of balls
        // First two are the enemy trench balls closest to our goal
        ballPosition[0] = new Translation2d(enemyTrenchBallX, enemyTrenchBall1Y);  
        ballPosition[1] = new Translation2d(enemyTrenchBallX, enemyTrenchBall2Y);
        // Next three are under the switch on edge closest to the far wall
        ballPosition[2] = new Translation2d(startLineX - 119.51 * Constants.inchToMetersConversionFactor, targetCenterPointY + 81.14 * Constants.inchToMetersConversionFactor);
        ballPosition[3] = new Translation2d(startLineX - 113.17 * Constants.inchToMetersConversionFactor, targetCenterPointY + 65.84 * Constants.inchToMetersConversionFactor);                            
        ballPosition[4] = new Translation2d(startLineX - 106.83 * Constants.inchToMetersConversionFactor, targetCenterPointY + 50.54 * Constants.inchToMetersConversionFactor);
        // Next 2 are under the switch on edge closest to our trench
        ballPosition[5] = new Translation2d(startLineX - 113.94 * Constants.inchToMetersConversionFactor, targetCenterPointY + 26.13 * Constants.inchToMetersConversionFactor);
        ballPosition[6] = new Translation2d(startLineX - 129.25 * Constants.inchToMetersConversionFactor, targetCenterPointY + 19.79 * Constants.inchToMetersConversionFactor);  
        // Next three are along the center of our trench from far wall back to control panel
        ballPosition[7] = new Translation2d(startLineX - 121.63 * Constants.inchToMetersConversionFactor, friendlyTrenchY);    
        ballPosition[8] = new Translation2d(startLineX - 157.63 * Constants.inchToMetersConversionFactor, friendlyTrenchY);     
        ballPosition[9] = new Translation2d(startLineX - 193.63 * Constants.inchToMetersConversionFactor, friendlyTrenchY);
        // Last two are under our control panel
        ballPosition[10] = new Translation2d(startLineX - 257.90 * Constants.inchToMetersConversionFactor, friendlyTrenchY + 9.25 * Constants.inchToMetersConversionFactor);
        ballPosition[11] = new Translation2d(startLineX - 257.90 * Constants.inchToMetersConversionFactor, friendlyTrenchY - 9.25 * Constants.inchToMetersConversionFactor);

    };
}
