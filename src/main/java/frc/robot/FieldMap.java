package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class FieldMap {

    // Robot dimensions with bumpers

    public static final double robotWidth = 34.75;
    public static final double robotLength = 37.5;

    // half dimensions - for getting the center point of the robot
    public static final double rW2 = robotWidth/2.0;
    public static final double rL2 = robotLength/2.0;
    
    // centerline is the robot starting position
    public static final double centerPoint = 227.42; // of target
    public static final double startLine = 509.145;
    public static final double friendlyTrenchX = 296.176; // center line through long axis of trench
    public static final double enemyTrenchBall1 = 19.574;       // Ball in enemy trench close to our goal
    public static final double enemyTrenchBall2 = 38.074;       // Other Ball in enemy trench close to our goal
    

    public static ArrayList<Translation2d> ballPositions = new ArrayList<Translation2d>();
    public static Pose2d[] startPositions = new Pose2d[3];
    // public static Translation2d[] controlPanel = new Translation2d[0];
    // public static Translation2d[] climberPoint = new Translation2d[2];

    public static ArrayList<Translation2d> wayPointsA = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsB = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsAlpha = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsBeta = new ArrayList<Translation2d>();
    public static ArrayList<Translation2d> wayPointsGamma = new ArrayList<Translation2d>();

    public FieldMap () {

        // start positions are in terms of the robot center (x,y)

        startPositions[1] = new Pose2d(centerPoint, startLine, new Rotation2d(0.0));        // directly centered
        startPositions[2] = new Pose2d(centerPoint - 35, startLine, new Rotation2d(0.0));   // offcenter by 35 inches
        startPositions[3] = new Pose2d(centerPoint + 35, startLine, new Rotation2d(0.0));   // offcenter by -35 inches
        startPositions[4] = new Pose2d(friendlyTrenchX, startLine, new Rotation2d(0.0));            // facing trench

        //locations of balls

        ballPositions.add(new Translation2d(friendlyTrenchX, 386.688));
        ballPositions.add(new Translation2d(friendlyTrenchX, 350.688));
        ballPositions.add(new Translation2d(friendlyTrenchX, 314.688));
        ballPositions.add(new Translation2d(enemyTrenchBall1, 378.959));


        /*
        
        YJ's original ball positions. We may add the rest of these later

        ballPosition[1] = new Translation2d(19.574, 378.959); 
        ballPosition[2] = new Translation2d(38.074, 378.959);
        ballPosition[3] = new Translation2d(146.281, 388.637);
        ballPosition[4] = new Translation2d(161.583, 394.975);
        ballPosition[5] = new Translation2d(176.884, 401.313);
        ballPosition[6] = new Translation2d(201.291, 394.200);
        ballPosition[7] = new Translation2d(207.630, 378.899);
        ballPosition[8] = new Translation2d(296.176, 386.688);
        ballPosition[9] = new Translation2d(296.176, 350.688);
        ballPosition[11] = new Translation2d(286.926, 250.417);
        ballPosition[12] = new Translation2d(305.426, 250.417);

        */

    };

}