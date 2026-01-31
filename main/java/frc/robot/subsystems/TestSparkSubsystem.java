package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSparkSubsystem extends SubsystemBase {

    
    private final SparkMax motor;
    private final SparkMax motor1;

    public TestSparkSubsystem() {
        motor = new SparkMax(15, MotorType.kBrushed);
        motor1 = new SparkMax(16, MotorType.kBrushed);
    }
    
    // motores lanzadores
    public void shooters() {
        motor.set(1); // 100%
        motor1.set(-1);
    }
    // motores recogedores
    public void intakers() {
        motor.set(-1); // 100%
        motor1.set(1);
    }

    public void stop0() {
        motor.stopMotor();
        motor1.stopMotor();
    }
}

