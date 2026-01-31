package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

    //  Puerto PWM (0–9). Cambia si usas otro
    private final PWMSparkMax shooterMotor;

    public ShooterSubsystem() {
        shooterMotor = new PWMSparkMax(15);

        shooterMotor.set(0.0); // motor apagado al iniciar
    }

    /** Potencia directa (0.0 a 1.0) */
    public void setPower(double power) {
        shooterMotor.set(power);
        System.out.println("Shooter DC power: " + power);
    }

    /** Apaga el motor */
    public void stop() {
        shooterMotor.set(0.0);
    }
}
