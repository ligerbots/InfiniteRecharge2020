package frc.robot;
import frc.robot.Constants;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class FieldMap {

    // Robot dimensions with bumpers

    public static final double robotWidth = 34.75 * Constants.inchToMetersConversionFactor;
    public static final double robotLength = 37.5 * Constants.inchToMetersConversionFactor;

    // half dimensions - for getting the center point of the robot
    public static final double rW2 = robotWidth/2.0;
    public static final double rL2 = robotLength/2.0;
    
    // centerline is the robot starting position
    public static final double targetCenterPointY = 92.341 * Constants.inchToMetersConversionFactor; // of target
    public static final double startLineX = (509.145 - 1) * Constants.inchToMetersConversionFactor + robotLength/2 ; //center the robot on the startpoint (Backedge of robot will line up with line)
    public static final double friendlyTrenchY = 25.414 * Constants.inchToMetersConversionFactor; // center line through long axis of trench
    public static final double enemyTrenchBall1Y = 302.016 * Constants.inchToMetersConversionFactor;       // Ball in enemy trench close to our goal
    public static final double enemyTrenchBall2Y = 283.516 * Constants.inchToMetersConversionFactor;       // Other Ball in enemy trench close to our goal
    public static final double bottomLeftCornerConversion = 325.00 * Constants.inchToMetersConversionFactor;
    
    public static ArrayList<Translation2d> ballPositions = new ArrayList<Translation2d>();
    public static Pose2d[] startPositions = new Pose2d[5];
    // public static Translation2d[] controlPanel = new Translation2d[0];
    // public static Translation2d[] climberPoint = new Translation2d[2];

    public static ArrayList<Translation2d> wayPointsA = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsB = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsAlpha = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsBeta = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsGamma = new ArrayList<Translation2d>();
    public static Translation2d[] ballPosition = new Translation2d[12];
    static {

        // start positions are in terms of the robot center (x,y)

        startPositions[0] = new Pose2d(startLineX, (bottomLeftCornerConversion - targetCenterPointY) ,  new Rotation2d(0.0));        // directly centered
        startPositions[1] = new Pose2d(startLineX, (bottomLeftCornerConversion - targetCenterPointY + 35 * Constants.inchToMetersConversionFactor), new Rotation2d(0.0));   // offcenter by 35 inches
        startPositions[2] = new Pose2d(startLineX, (bottomLeftCornerConversion - targetCenterPointY - 35 * Constants.inchToMetersConversionFactor), new Rotation2d(0.0));   // offcenter by -35 inches
        startPositions[3] = new Pose2d(startLineX, (bottomLeftCornerConversion - friendlyTrenchY) ,  new Rotation2d(0.0));            // facing trench
        startPositions[4] = new Pose2d(startLineX, (bottomLeftCornerConversion - 220.341 * Constants.inchToMetersConversionFactor), new Rotation2d(0.0)); // centered in front of enemy feeder station
        //locations of balls
        //Positions of balls 1-12 on the field
        ballPosition[0] = new Translation2d(378.959 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - enemyTrenchBall1Y));  
        ballPosition[1] = new Translation2d(378.959 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - enemyTrenchBall2Y));
        ballPosition[2] = new Translation2d(388.637 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 173.480 * Constants.inchToMetersConversionFactor)); 
        ballPosition[3] = new Translation2d(394.975 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 158.178 * Constants.inchToMetersConversionFactor));                            
        ballPosition[4] = new Translation2d(401.313 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 142.877 * Constants.inchToMetersConversionFactor));
        ballPosition[5] = new Translation2d(394.200 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 118.470 * Constants.inchToMetersConversionFactor));
        ballPosition[6] = new Translation2d(378.899 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 112.131 * Constants.inchToMetersConversionFactor));  
        ballPosition[7] = new Translation2d(386.688 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - friendlyTrenchY));    
        ballPosition[8] = new Translation2d(350.688 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - friendlyTrenchY));     
        ballPosition[9] = new Translation2d(314.688 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - friendlyTrenchY));
        ballPosition[10] = new Translation2d(250.417 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 34.664 * Constants.inchToMetersConversionFactor));
        ballPosition[11] = new Translation2d(250.417 * Constants.inchToMetersConversionFactor,(bottomLeftCornerConversion - 16.164 * Constants.inchToMetersConversionFactor));

        
        ballPositions.add(ballPosition[1]);
        ballPositions.add(ballPosition[2]);
        ballPositions.add(ballPosition[8]);
        ballPositions.add(ballPosition[9]);
        ballPositions.add(ballPosition[10]);
    };
}