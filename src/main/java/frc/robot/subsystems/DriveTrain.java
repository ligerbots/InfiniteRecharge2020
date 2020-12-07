/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.FieldMap;

// For simulation
import frc.robot.simulation.SparkMaxWrapper;
import frc.robot.simulation.AHRSSimWrapper;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.Field2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax leftLeader = new SparkMaxWrapper(Constants.LEADER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax leftFollower = new SparkMaxWrapper(Constants.FOLLOWER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax rightLeader = new SparkMaxWrapper(Constants.LEADER_RIGHT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax rightFollower = new SparkMaxWrapper(Constants.FOLLOWER_RIGHT_CAN_ID, MotorType.kBrushless);

    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLeader, leftFollower);

    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLeader, rightFollower);

    // public PIDController turnSpeedController;

    public double turnOutput;

    DifferentialDrive robotDrive;
    DifferentialDriveOdometry odometry;

    Encoder leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    Encoder rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    AHRS navX;

    double limitedThrottle;

    // Simulation classes
    public DifferentialDrivetrainSim drivetrainSimulator;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    // The Field2d class simulates the field in the sim GUI. Note that we can have only one
    // instance!
    private Field2d fieldSim;
    private SimDouble gyroAngleSim;

    private int prevBallLocation = 0;
    private int prevStartLocation = 10;

    public DriveTrain() {

        // TODO: Verify which motors need to be inverted
        // Since we're using DifferentialDrive below, we should not need to invert any,
        // but it doesn't hurt to be explicit.

        robotDrive = new DifferentialDrive(leftMotors, rightMotors);
        robotDrive.setSafetyEnabled(false);

        navX = new AHRSSimWrapper(Port.kMXP, (byte) 200);

        // Set current limiting on drve train to prevent brown outs
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        //TODO determine real numbers to use here
        //rightLeader.setOpenLoopRampRate(0.0065);
        //leftLeader.setOpenLoopRampRate(0.0065);

        ////////////////////////////ODOMETRY SET UP//////////////////////////////////

        leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

        // turnSpeedController = new PIDController(0.015, 0.0001, 0.0, 0, navX, output -> this.turnOutput = output);

        if (RobotBase.isSimulation()) {
            // If our robot is simulated
            // This class simulates our drivetrain's motion around the field.
            drivetrainSimulator = new DifferentialDrivetrainSim(
                  Constants.kDrivetrainPlant,
                  Constants.kDriveGearbox,
                  Constants.kDriveGearing,
                  Constants.kTrackwidth,
                  Constants.kWheelDiameterMeters / 2.0);
      
            // The encoder and gyro angle sims let us set simulated sensor readings
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            
            // get the angle simulation variable
            // SimDevice is found by name and index, like "name[index]"
            gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value + "]").getDouble("Angle");
      
            // the Field2d class lets us visualize our robot in the simulation GUI.
            fieldSim = new Field2d();

            SmartDashboard.putNumber("moveAroundField/startPos", prevStartLocation);
            SmartDashboard.putNumber("moveAroundField/ballPos", prevBallLocation);
        }

        // TODO this should not be here
        SmartDashboard.putString("vision/active_mode/selected", "goalfinder");
    }

    public Pose2d getPose () {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            // This is a bit hokey, but if the Robot jumps on the field, we need
            //   to reset the internal state of the DriveTrainSimulator.
            //   No method to do it, but we can reset the state variables.
            //   NOTE: this assumes the robot is not moving, since we are not resetting
            //   the rate variables.
            drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

            // reset the GyroSim to match the driveTrainSim
            // do it early so that "real" odometry matches this value
            gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
            fieldSim.setRobotPose(pose);
        }

        // The left and right encoders MUST be reset when odometry is reset
        leftEncoder.reset();
        rightEncoder.reset();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
    }
      
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        leftMotors.setVoltage(-leftVolts);
        rightMotors.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
        robotDrive.feed();
    }
    
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }
    
    public double getLeftEncoderDistance() {
        return leftEncoder.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return rightEncoder.getDistance();
    }
    
    public double getHeading() {
        return odometry.getPoseMeters().getRotation().getDegrees();
      }
    
    private double getGyroAngle() {
        return Math.IEEEremainder(navX.getAngle(), 360) * -1; // -1 here for unknown reason look in documatation
    }

    // This is the same as setPose() but leave here for compatibility
    public void resetOdometry (Pose2d pose) {
        setPose(pose);
        // old code as in master branch. leave here until tested
        // resetEncoders();
        // //resetHeading();
        // odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));

        // if (RobotBase.isSimulation()) {
        //     fieldSim.setRobotPose(pose);
        // }
    }

    // This is not used, but leave it here for future work?
    //
    // public void enableTurningControl(double angle, double tolerance) {
    //     //double angleOffset = angle;
    //     double startAngle = getHeading();
    //     double targetAngle = startAngle + angle;
    
    //     // We need to keep all angles between -180 and 180. Account for that here
    //     // wrapCorrection will be used below in turnError to undo what we do here
    //     double originalTargetAngle = targetAngle;
    //     if (targetAngle > 180.0) {
    //         targetAngle -= 360.0;
    //     }
    //     else if (targetAngle < -180.0) {
    //         targetAngle += 360.0;
    //     }
        
    //     turnSpeedController.setSetpoint(targetAngle);
    //     turnSpeedController.enable();
    //     turnSpeedController.setInputRange(-180.0, 180.0);
    //     turnSpeedController.setAbsoluteTolerance(tolerance);
    //     turnSpeedController.setOutputRange(-1.0, 1.0);
    //     turnSpeedController.setContinuous(true);

    //     System.out.printf(
    //         "currentAngle: %5.2f, originalTargetAngle: %5.2f, targetAngle: %5.2f, ",
    //         startAngle, originalTargetAngle, targetAngle);
    // }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getGyroAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Pose", getPose().toString());
    }

    @Override
    public void simulationPeriodic() {
      // To update our simulation, we set motor voltage inputs, update the simulation,
      // and write the simulated positions and velocities to our simulated encoder and gyro.
      // We negate the right side so that positive voltages make the right side
      // move forward.
      drivetrainSimulator.setInputs(-leftMotors.get() * RobotController.getBatteryVoltage(),
                                    rightMotors.get() * RobotController.getBatteryVoltage());
      drivetrainSimulator.update(0.020);

      leftEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftPosition));
      leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftVelocity));
  
      rightEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightPosition));
      rightEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightVelocity));
  
      gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
  
      fieldSim.setRobotPose(getPose());
    }

    public void allDrive(double throttle, double rotate, boolean squaredInputs) {
        // TODO: We should look into using the deadband settings in DifferentialDrive
        if (squaredInputs) {
            if (Math.abs(throttle) < 0.1)
                throttle = 0;
            if (Math.abs(rotate) < 0.1) 
                rotate = 0;
        }
        robotDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    }

    public int getLeftEncoderTicks () {
        return leftEncoder.get();
    }

    public int getRightEncoderTicks () {
        return rightEncoder.get();
    }

    public double turnSpeedCalc(double angleError) {
        if (Math.abs(angleError) > 60) {
            return 0.8 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 30) {
            return 0.4 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 10) {
            return 0.13 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 5) {
            return 0.07 * Math.signum(angleError);
        }
        else {
            return 0.065 * Math.signum(angleError);
        }
    }

    public double getPitch() {
        return navX.getPitch();
    }

    public void setIdleMode(IdleMode idleMode) {
        if (Robot.isReal()) {
            Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));
        }
    }

    public void moveAroundField() {
        // only applies for simulation
        if (RobotBase.isReal()) return;

        int startPos = (int)SmartDashboard.getNumber("moveAroundField/startPos", 10);
        int ballPos = (int)SmartDashboard.getNumber("moveAroundField/ballPos", 0);

        // Use either start of ball to set robot pose.
        // 10 is the dummy default value for start.
        if (startPos != prevStartLocation && startPos >= 0 && startPos < FieldMap.startPosition.length) {
            // The start value has changed and is valid. Use it to position the robot.
            fieldSim.setRobotPose(FieldMap.startPosition[startPos]);
        } else if (ballPos != prevBallLocation && ballPos >= 0 && ballPos < FieldMap.ballPosition.length) {
            // start value is invalid, so use the ball position with 0 rotation angle
            fieldSim.setRobotPose(new Pose2d(FieldMap.ballPosition[ballPos], new Rotation2d(0.0)));
        }

        prevBallLocation = ballPos;
        prevStartLocation = startPos;

        // On every call, output the Pose info to SmartDashboard for debugging convenience
        Pose2d pose = fieldSim.getRobotPose();
        SmartDashboard.putNumber("moveAroundField/robotX", pose.getX() / Constants.inchToMetersConversionFactor);
        SmartDashboard.putNumber("moveAroundField/robotY", pose.getY() / Constants.inchToMetersConversionFactor);
        SmartDashboard.putNumber("moveAroundField/robotAngle", pose.getRotation().getDegrees());
    }

    public void setRobotFromFieldPose() {
        // only applies for simulation
        if (RobotBase.isSimulation())
            setPose(fieldSim.getRobotPose());
    }
}
