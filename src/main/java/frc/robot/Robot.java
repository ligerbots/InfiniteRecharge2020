/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.Carousel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private AutoCommandInterface m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  SendableChooser<AutoCommandInterface> chosenAuto = new SendableChooser<>();

  public static int RPMAdjustment;
  public static int HoodAdjustment;
  public static double angleErrorAfterTurn = 0;

  private AutoCommandInterface m_prevAutoCommand = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   // This is just a comment to demonstrate commits in Git.
   
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);

    // Set motors to coast so it's easier to move the robot.
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);

    //m_robotContainer.shooter.calibratePID(0.000085, 0.000000033, 0, 6.776 * 0.00001);

    // Reset Smart Dashboard for shooter test
    SmartDashboard.putString("Shooting", "Idle");

    RPMAdjustment = 0;
    HoodAdjustment = 0;

    // SmartDashboard.putData(new TestTurret(m_robotContainer.shooter));
    /*
    chosenAuto.addDefault("Default Auto", new DriveForwardAuto(m_robotContainer.robotDrive, m_robotContainer.carouselCommand, m_robotContainer.driveCommand));


    chosenAuto.addObject("EightBallAuto", new EightBallAuto(
      m_robotContainer.robotDrive,
      m_robotContainer.shooter,
      m_robotContainer.intake,
      m_robotContainer.climber,
      m_robotContainer.carousel,
      m_robotContainer.driveCommand,
      m_robotContainer.carouselCommand));

    chosenAuto.addObject("ShootAndDrive", new ShootAndDriveAuto(
        m_robotContainer.robotDrive,
        m_robotContainer.shooter,
        m_robotContainer.intake,
        m_robotContainer.climber,
        m_robotContainer.carousel,
        m_robotContainer.driveCommand,
        m_robotContainer.carouselCommand));
    */

    chosenAuto.setDefaultOption("NewEightBallSim", new NewEightBallSim(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.climber, m_robotContainer.carousel, m_robotContainer.carouselCommand, m_robotContainer.shooter));
    chosenAuto.addOption("MoveAroundField", new MoveAroundField());
    chosenAuto.addOption("TrenchAuto Pos 0", new TrenchAuto(FieldMap.startPosition[0], m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.climber, m_robotContainer.carousel, m_robotContainer.carouselCommand, m_robotContainer.shooter));
    chosenAuto.addOption("TrenchAuto Pos 2", new TrenchAuto(FieldMap.startPosition[2], m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.climber, m_robotContainer.carousel, m_robotContainer.carouselCommand, m_robotContainer.shooter));
    chosenAuto.addOption("ShedTest", new ShedTest(m_robotContainer.robotDrive));
    SmartDashboard.putData("Chosen Auto", chosenAuto);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    if (Robot.isReal()) {
      m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);
      m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);
      // Set motors to coast so it's easier to move the robot.
      m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);
      m_robotContainer.climber.coastWinch();
    }
  }

  @Override
  public void disabledPeriodic() {
    //m_robotContainer.carouselCommand.schedule();

    // Do not use the member variable m_autonomousCommand. Setting that signals
    //  that the command is running, which it is not, yet.
    AutoCommandInterface autoCommandInterface = chosenAuto.getSelected();
    if (autoCommandInterface != null && autoCommandInterface != m_prevAutoCommand) {
      m_robotContainer.robotDrive.setPose(autoCommandInterface.getInitialPose());
      m_prevAutoCommand = autoCommandInterface;
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    m_robotContainer.carousel.resetEncoder();
    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    //m_robotContainer.carouselCommand.schedule();

    // schedule the autonomous command
    m_autonomousCommand = chosenAuto.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // Do this immediately before changing any motor settings, etc.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("Turret Angle", 75);
    SmartDashboard.putNumber("Target Hood Angle", 60);
    SmartDashboard.putNumber("TSR", -5500);
    System.out.println("teleopInit");

    // Reset the winch encoder
    m_robotContainer.climber.resetWinchEncoder();
    m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);

    //SmartDashboard.putData(m_robotContainer.testFlup);

    m_robotContainer.driveCommand.schedule();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.shooter.testSpin();
    m_robotContainer.carouselCommand.schedule();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.testIntake.schedule();
    //RunWinch aaa = new RunWinch(m_robotContainer.climber, m_robotContainer);
    //aaa.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.moveAroundField();
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
