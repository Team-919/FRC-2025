// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

//import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/** Class to run the rollers over CAN using Vector SPX */
public class CANRollerSubsystem extends SubsystemBase {

  //private final SparkMax rollerMotor;
  private final WPI_VictorSPX rollMotor;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    //rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    rollMotor = new WPI_VictorSPX(RollerConstants.ROLLER_MOTOR_ID);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //rollerMotor.setCANTimeout(250);
    //rollMotor.wait(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    //VictorSPXConfiguration rollConfig = new VictorSPXConfiguration();
    
    //SparkMaxConfig rollerConfig = new SparkMaxConfig();
    //rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    //rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    //rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Print the current position of the roller for debugging
  }

  /** Runs the roller at a manual speed */
  public void runRoller(double forward, double reverse) {
    //rollerMotor.set(forward - reverse);
    rollMotor.set(forward-reverse);

  }

  public void setVoltage(double voltage) {
    rollMotor.set(voltage);
  }
}
