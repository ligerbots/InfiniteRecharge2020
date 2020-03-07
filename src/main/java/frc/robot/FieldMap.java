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
    public static final double targetCenterPointY = 92.341; // of target
    public static final double startLineX = 509.145 + robotLength/2 - 1; //center the robot on the startpoint (Backedge of robot will line up with line)
    public static final double friendlyTrenchY = 25.414; // center line through long axis of trench
    public static final double enemyTrenchBall1Y = 302.016;       // Ball in enemy trench close to our goal
    public static final double enemyTrenchBall2Y = 283.516;       // Other Ball in enemy trench close to our goal
    

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
    public FieldMap () {

        // start positions are in terms of the robot center (x,y)

        startPositions[1] = new Pose2d(startLineX, targetCenterPointY,  new Rotation2d(0.0));        // directly centered
        startPositions[2] = new Pose2d(startLineX, targetCenterPointY - 35, new Rotation2d(0.0));   // offcenter by 35 inches
        startPositions[3] = new Pose2d(startLineX, targetCenterPointY + 35, new Rotation2d(0.0));   // offcenter by -35 inches
        startPositions[4] = new Pose2d(startLineX, friendlyTrenchY,  new Rotation2d(0.0));            // facing trench
        startPositions[5] = new Pose2d(startLineX, 220.341, new Rotation2d(0.0)); // centered in front of enemy feeder station
        //locations of balls
        //Positions of balls 1-12 on the field
        ballPosition[1] = new Translation2d( 378.959, enemyTrenchBall1Y);  
        ballPosition[2] = new Translation2d( 378.959, enemyTrenchBall2Y);
        ballPosition[3] = new Translation2d(388.637, 173.480); 
        ballPosition[4] = new Translation2d(394.975, 158.178);                            
        ballPosition[5] = new Translation2d(401.313, 142.877 );
        ballPosition[6] = new Translation2d(394.200, 118.470);
        ballPosition[7] = new Translation2d(378.899, 112.131);  
        ballPosition[8] = new Translation2d(386.688, friendlyTrenchY);    
        ballPosition[9] = new Translation2d(350.688, friendlyTrenchY);     
        ballPosition[10] = new Translation2d(314.688, friendlyTrenchY);
        ballPosition[11] = new Translation2d(250.417, 34.664);
        ballPosition[12] = new Translation2d(250.417,16.164);

        
        ballPositions.add(ballPosition[1]);
        ballPositions.add(ballPosition[2]);
        ballPositions.add(ballPosition[8]);
        ballPositions.add(ballPosition[9]);
        ballPositions.add(ballPosition[10]);

        
        

    };

}