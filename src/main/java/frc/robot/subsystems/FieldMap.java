package frc.robot.subsystems;



import java.util.ArrayList;

import java.util.LinkedList;

import java.util.List;
@SuppressWarnings("all")




public class FieldMap {

    

    

    // Robot dimensions with bumpers

    public static final double robotWidth = 0; // TODO: Determine final robot width and length

    public static final double robotLength = 0;

    // half dimensions - for getting the center point of the robot

    public static final double rW2 = robotWidth/2.0;

    public static final double rL2 = robotLength/2.0;

    public static final double centerLine = 227.42;
    

    public static FieldPosition[] startPositions = new FieldPosition[5];   // position 0 is not used! (because the diagram is 1-based)

    public static FieldPosition[] ballPosition = new FieldPosition[12];

    public static FieldPosition[] controlPanel = new FieldPosition[0];

    public static FieldPosition[] climberPoint = new FieldPosition[2];
    // The positions for our scoring positions on all platforms is symmetric around the X axis
   

    
    // Way Points are the intermediate points on the way to a particular scoring position

    // They are meant to be universal -- the first point should be something that can be

    // reached in a straight line from any starting position. In practice, some might not

    // work so well. We depend on the user to not choose a waypoint that would drive

    // clear across the field, or that is likely to collide with an alliance partner

    public static ArrayList<FieldPosition> wayPointsA = new ArrayList<FieldPosition>();

    public static ArrayList<FieldPosition> wayPointsB = new ArrayList<FieldPosition>();

    public static ArrayList<FieldPosition> wayPointsAlpha = new ArrayList<FieldPosition>();

    public static ArrayList<FieldPosition> wayPointsBeta = new ArrayList<FieldPosition>();

    public static ArrayList<FieldPosition> wayPointsGamma = new ArrayList<FieldPosition>();

   
    

    public FieldMap () {

        // start positions are in terms of the robot center (x,y)

        startPositions[1] = new FieldPosition(centerLine, 509.145);  // directly centered

        startPositions[2] = new FieldPosition(centerLine - 35, 509.145);   // offcenter by 35 inches

        startPositions[3] = new FieldPosition(centerLine + 35, 509.145);   // offcenter by -35 inches

        startPositions[4] = new FieldPosition(296.176, 509.145);  // facing trench

        

        

        // TODO: configure raw x,y coordinates to center robot
        //locations of balls

        ballPosition[0] = new FieldPosition(19.574, 378.959);
        ballPosition[1] = new FieldPosition(38.074, 378.959);
        ballPosition[2] = new FieldPosition(146.281, 388.637);
        ballPosition[3] = new FieldPosition(161.583, 394.975);
        ballPosition[4] = new FieldPosition(176.884, 401.313);
        ballPosition[5] = new FieldPosition(201.291, 394.200);
        ballPosition[6] = new FieldPosition(207.630, 378.899);
        ballPosition[7] = new FieldPosition(296.176, 386.688);
        ballPosition[8] = new FieldPosition(296.176, 350.688);
        ballPosition[9] = new FieldPosition(296.176, 314.688);
        ballPosition[10] = new FieldPosition(286.926, 250.417);
        ballPosition[11] = new FieldPosition(305.426, 250.417);
        
        //location of control panel
        controlPanel[0] = new FieldPosition(296.176, 269.642);

        //Locations of climbers
        
        climberPoint[0] = new FieldPosition(195.882, 331.763);
        climberPoint[1] = new FieldPosition(211.057, 298.549);
        climberPoint[2] = new FieldPosition(181.062, 365.502);


        // TODO -- create waypoints corresponding to each scoring position

        //         that will ensure the robot doesn't crash into things

        //         or enter forbidden areas

        // Waypoints are just field positions. Don't need angles.

        

        // There's an implicit waypoint for all robots 8" out from their starting position

        // to ensure there's enough space for them to rotate without hitting the back wall

        

       /* wayPointsA.add(new FieldPosition(45.0, 60));

        wayPointsA.add(switchScoringSpot[0]);

        

        wayPointsB.add(new FieldPosition(126.0, 120.0, 4.0));

        wayPointsB.add(new FieldPosition(126.0, 150.0));

        wayPointsB.add(switchScoringSpot[1]);

        

        wayPointsAlpha.add(new FieldPosition(117.0, 120.0, 4.0));

        wayPointsAlpha.add(new FieldPosition(117.0, 185.0, FieldMap.scaleScoringHeight));

        wayPointsAlpha.add(scaleScoringSpot[0]);



        wayPointsBeta.add(new FieldPosition(128.0, 165.0, 4.0));

        wayPointsBeta.add(new FieldPosition(128.0, 240.0));

        wayPointsBeta.add(new FieldPosition(130.0, 310.0));

        wayPointsBeta.add(scaleScoringSpot[1]);

        

        wayPointsGamma.add(new FieldPosition(117.5, 180.0, 4.0));

        wayPointsGamma.add(new FieldPosition(-80, 180));

        wayPointsGamma.add(scaleScoringSpot[2]); */

        



        



    }
  }

    

  

    

    /*public static List<FieldPosition> generateCatmullRomSpline(List<FieldPosition> controlPoints) {

      LinkedList<FieldPosition> output = new LinkedList<>();



      for (int i = 1; i < controlPoints.size() - 2; i++) {

        FieldPosition p0 = controlPoints.get(i - 1);

        FieldPosition p1 = controlPoints.get(i);

        FieldPosition p2 = controlPoints.get(i + 1);

        FieldPosition p3 = controlPoints.get(i + 2);



        generateSegment(p0, p1, p2, p3, output, i == 1);

      }



      return output;

    }



    /**

     * Generates a Catmull-Rom spline segment.

     * 

     * @param p0 Control point

     * @param p1 Control point

     * @param p2 Control point

     * @param p3 Control point

     * @param output The list to add points to

     */

   /*private static void generateSegment(FieldPosition p0, FieldPosition p1, FieldPosition p2,

        FieldPosition p3, List<FieldPosition> output, boolean isFirst) {

      int numPoints = (int) Math.ceil(p1.distanceTo(p2) / 4.0);

      if (numPoints < 5) {

        numPoints = 5;

      }



      double t0 = 0;

      double t1 = calculateT(t0, p0, p1);

      double t2 = calculateT(t1, p1, p2);

      double t3 = calculateT(t2, p2, p3);



      double deltaT = Math.abs(t2 - t1) / (numPoints - 1);



      for (int i = isFirst ? 0 : 1; i < numPoints; i++) {

        double ti = i * deltaT + t1;

        FieldPosition a1 = p0.multiply((t1 - ti) / (t1 - t0)).add(p1.multiply((ti - t0) / (t1 - t0)));

        FieldPosition a2 = p1.multiply((t2 - ti) / (t2 - t1)).add(p2.multiply((ti - t1) / (t2 - t1)));

        FieldPosition a3 = p2.multiply((t3 - ti) / (t3 - t2)).add(p3.multiply((ti - t2) / (t3 - t2)));



        FieldPosition b1 = a1.multiply((t2 - ti) / (t2 - t0)).add(a2.multiply((ti - t0) / (t2 - t0)));

        FieldPosition b2 = a2.multiply((t3 - ti) / (t3 - t1)).add(a3.multiply((ti - t1) / (t3 - t1)));



        output.add(b1.multiply((t2 - ti) / (t2 - t1)).add(b2.multiply((ti - t1) / (t2 - t1))));

      }

    }



    private static final double alpha = 0.5;



    private static double calculateT(double ti, FieldPosition p0, FieldPosition p1) {

      double x0 = p0.x;

      double y0 = p0.y;

      double x1 = p1.x;

      double y1 = p1.y;



      double dx = x1 - x0;

      double dy = y1 - y0;



      return Math.pow(Math.sqrt(dx * dx + dy * dy), alpha) + ti;

    }

    







}*/