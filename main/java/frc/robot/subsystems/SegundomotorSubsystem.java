package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SegundomotorSubsystem extends SubsystemBase {

    private final SparkMax motor_paro;

    public SegundomotorSubsystem() {
        motor_paro = new SparkMax(20, MotorType.kBrushed);
    
    }

    public void run() {
        motor_paro.set(-1); // 100%
    }

    public void run_vice() {
        motor_paro.set(-1); // 100%
    }

    public void stop() {
        motor_paro.stopMotor();
    }
}
