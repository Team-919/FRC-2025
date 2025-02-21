package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveSparkMax;
    private final SparkMax turningSparkMax;
    
    public SwerveModule(int driveId, int turningId, double anuglarOffset){
        driveSparkMax = new SparkMax(driveId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningId, MotorType.kBrushless);

        
    }
}