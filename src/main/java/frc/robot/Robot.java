// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> DriveSubsystem.m_frontLeft.m_turningEncoder.getPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> DriveSubsystem.m_frontLeft.m_drivingEncoder.getVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> DriveSubsystem.m_frontRight.m_turningEncoder.getPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> DriveSubsystem.m_frontRight.m_drivingEncoder.getVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> DriveSubsystem.m_rearLeft.m_turningEncoder.getPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> DriveSubsystem.m_rearLeft.m_drivingEncoder.getVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> DriveSubsystem.m_rearRight.m_turningEncoder.getPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> DriveSubsystem.m_rearRight.m_drivingEncoder.getVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> DriveSubsystem.m_gyro.getAngle(), null);
      }
    });
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_field.setRobotPose(DriveSubsystem.m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Max Speed", Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Gyro", DriveSubsystem.m_gyro.getAngle());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
 
  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //Go Straight and Spew L1 STARTING IN MIDDLE
    m_autonomousCommand = m_robotContainer.Middle();

    //Go Straight STARTING ON SIDES
    //m_autonomousCommand = m_robotContainer.Side();

  //m_autonomousCommand = m_robotContainer.SideL1Scoring();



    //Go Straight and spew L1 180 turn (middle position)
    //m_autonomousCommand = m_robotContainer.auto2();

    //dont move
    //m_autonomousCommand = m_robotContainer.standStill();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
