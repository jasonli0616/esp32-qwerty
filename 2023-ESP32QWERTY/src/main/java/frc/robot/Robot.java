// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.kDrivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  AnalogInput yAxis;
  AnalogInput xAxis;

  WPI_TalonSRX m_frontLeftMotor;
  WPI_TalonSRX m_frontRightMotor;
  WPI_TalonSRX m_rearLeftMotor;
  WPI_TalonSRX m_rearRightMotor;

  DifferentialDrive m_differentialDrive;

  int tick = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    yAxis = new AnalogInput(0);
    xAxis = new AnalogInput(1);

    m_frontLeftMotor = new WPI_TalonSRX(kDrivetrain.kMotors.frontLeftMotorID); //*
    m_frontRightMotor = new WPI_TalonSRX(kDrivetrain.kMotors.frontRightMotorID);
    m_rearLeftMotor = new WPI_TalonSRX(kDrivetrain.kMotors.rearLeftMotorID);
    m_rearRightMotor = new WPI_TalonSRX(kDrivetrain.kMotors.rearRightMotorID);

    m_rearLeftMotor.follow(m_frontLeftMotor);
    m_rearRightMotor.follow(m_frontRightMotor);

    m_frontLeftMotor.setInverted(true);
    m_frontRightMotor.setInverted(false);
    m_rearLeftMotor.setInverted(true);
    m_rearRightMotor.setInverted(false);

    m_frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    m_frontRightMotor.setNeutralMode(NeutralMode.Brake);
    m_rearLeftMotor.setNeutralMode(NeutralMode.Brake);
    m_rearRightMotor.setNeutralMode(NeutralMode.Brake);

    m_frontLeftMotor.configOpenloopRamp(kDrivetrain.kMotors.rampRate);
    m_frontRightMotor.configOpenloopRamp(kDrivetrain.kMotors.rampRate);
    m_rearLeftMotor.configOpenloopRamp(kDrivetrain.kMotors.rampRate);
    m_rearRightMotor.configOpenloopRamp(kDrivetrain.kMotors.rampRate);

    m_differentialDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double xAxisValue = MathUtil.applyDeadband(mapRange(xAxis.getAverageVoltage(), 0, 3.2, -1, 1), 0.4);
    double yAxisValue = -MathUtil.applyDeadband(mapRange(yAxis.getAverageVoltage(), 0, 3.2, -1, 1), 0.4);

    SmartDashboard.putNumber("Y", yAxisValue);
    SmartDashboard.putNumber("X", xAxisValue);

    m_differentialDrive.arcadeDrive(xAxisValue, yAxisValue);

    tick++;

    if (tick % 50 == 0) {
      yAxis.resetAccumulator();
      xAxis.resetAccumulator();
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  double mapRange(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
