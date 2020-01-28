/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TALON IDs
    public static final int LEADER_LEFT_TALON_ID = 1;

    public static final int LEADER_RIGHT_TALON_ID = 2;

    public static final int FOLLOWER_LEFT_TALON_ID = 3;

    public static final int FOLLOWER_RIGHT_TALON_ID = 4;

    public static final int[] LEFT_ENCODER_PORTS = new int[]{1, 2};
    
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{3, 4};

    public static final double DISTANCE_PER_PULSE = 0.0001; // TODO: find real number ~ This is the coefficient of encoder ticks to convert to distance
    

}