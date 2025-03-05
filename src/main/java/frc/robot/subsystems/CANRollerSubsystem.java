// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.vector.sbx.SBXMotor;

/** Class to run the rollers over CAN using Vector SBX */
public class CANRollerSubsystem extends SubsystemBase {
  private final SBXMotor rollerMotor;
  private final SBXEncoder rollerEncoder;
  private final SBXPIDController pidController;

  public CANRollerSubsystem() {
    // Initialize the SBX motor with the correct CAN ID
    rollerMotor = new SBXMotor(RollerConstants.ROLLER_MOTOR_ID);

    // Initialize encoder from the motor
    rollerEncoder = rollerMotor.getEncoder();

    // Initialize PID controller
    pidController = rollerMotor.getPIDController();

    // Set brake mode (assuming roller needs to stop when power is cut)
    rollerMotor.setBrakeMode(true);

    // Set voltage compensation
    rollerMotor.setVoltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);

    // Set current limit to protect the motor
    rollerMotor.setCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);

    // Configure PID controller (tune these values in Constants)
    pidController.setP(RollerConstants.kP);
    pidController.setI(RollerConstants.kI);
    pidController.setD(RollerConstants.kD);
    pidController.setFF(RollerConstants.kFF);
  }

  @Override
  public void periodic() {
    // Print the current position of the roller for debugging
    System.out.println("Roller Position: " + rollerEncoder.getPosition());
  }

  /** Runs the roller at a manual speed */
  public void runRoller(double forward, double reverse) {
    rollerMotor.setSpeed(forward - reverse);
  }

  /** Runs the roller to a set position using PID */
  public void moveToPosition(double position) {
    pidController.setReference(position);
  }

  /** Stops the roller */
  public void stopRoller() {
    rollerMotor.setSpeed(0);
  }

  /** Resets the encoder value */
  public void resetEncoder() {
    rollerEncoder.reset();
  }
}
