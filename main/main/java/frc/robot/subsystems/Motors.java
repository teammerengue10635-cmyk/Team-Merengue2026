package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motors extends SubsystemBase {

    private final SparkMax Motorshooter1;
    private final SparkMax Motorshooter2;
    private final SparkMax intakeMotor;

    // ===== MOTOR 45° =====
    private final SparkMax motor45;
    private final SparkClosedLoopController motor45Controller;
    private final SparkMax motor045;
    private final SparkClosedLoopController motor045Controller;
    @SuppressWarnings("removal")
    public Motors() {
        Motorshooter1 = new SparkMax(15, MotorType.kBrushed);
        Motorshooter2 = new SparkMax(16, MotorType.kBrushed);
        intakeMotor = new SparkMax(17, MotorType.kBrushless);

        // ===== MOTOR 45° =====
        motor45 = new SparkMax(18, MotorType.kBrushed);
        motor45Controller = motor45.getClosedLoopController();

        motor045 = new SparkMax(19, MotorType.kBrushed);
        motor045Controller = motor045.getClosedLoopController();

        SparkMaxConfig config45 = new SparkMaxConfig();
        config45.closedLoop.pid(0.2, 0.0, 0.0);
        
        SparkMaxConfig config045 = new SparkMaxConfig();
        config045.closedLoop.pid(0.2, 0.0, 0.0);

        motor45.configure(
            config45,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        motor45.getEncoder().setPosition(0);
        
        motor045.configure(
            config045,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        motor045.getEncoder().setPosition(0);
    }
    
    

    /* ================= Shooters ================= */

    public void shooters() {
        Motorshooter1.set(-1);
        Motorshooter2.set(1);
    }
   
    public void stopShooter() {
        Motorshooter1.stopMotor();
        Motorshooter2.stopMotor();
    }

    /* ================= Intakers ================= */

    public void intakers() {
        intakeMotor.set(-1);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    /* ================= primer MOTOR 45° ================= */

    private double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    public void moveTo45Right() {
        motor45Controller.setSetpoint(degreesToRotations(45), ControlType.kPosition
        );
       
    }

    public void moveTo45Left() {
        motor45Controller.setSetpoint(degreesToRotations(-45), ControlType.kPosition
        );
    }

    public void stopMotor45() {
        motor45.stopMotor();
    }

             // Segundo motor de 45

     public void moveTo045Right() {
          motor045Controller.setSetpoint(degreesToRotations(-45), ControlType.kPosition
        );
     
    }

    public void moveTo045Left() {
         motor045Controller.setSetpoint(degreesToRotations(45), ControlType.kPosition
        );
    }

    public void stopMotor045() {
        motor045.stopMotor();
    }

    /* ================= GENERAL STOP ================= */

    public void stopAll() {
        Motorshooter1.stopMotor();
        Motorshooter2.stopMotor();
        intakeMotor.stopMotor();
        motor45.stopMotor();
        motor045.stopMotor();  
    }
}