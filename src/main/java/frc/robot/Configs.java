package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // use module constants to calculate conversion factors and feed forward gain
            // ex. driv factor converts motor rotations to metres
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // encoder ticks to metres
                .velocityConversionFactor(drivingFactor / 60.0); // encoder velocity to m/s
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // may need to change, higher p reacts faster but can cause unwanted back and forth movement
                // integral is drift correction derivitave is reduces sudden changes
                .pid(0.08, 0, 0)//okok
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                // invert turning encoder because output shaft rotates in the opposite direction of the steering motor in MAXSwerve module
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // may need to change
                .pid(5, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1)
                // PID wraps around for turning motor, eg go thru 0
                // pid controller going from 350 to 10 degrees will go thru 0 rather backwards, which will take longer
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }

        public Command autoBalanceCommand() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'autoBalanceCommand'");
        }
    }
}
