package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
/* import com.revrobotics.spark.SparkBase.ControlType; */
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class Motors extends SubsystemBase {

    private final DutyCycleOut shooterRequest = new DutyCycleOut(0);
    private final DutyCycleOut intakerRequest = new DutyCycleOut(0);
    
    private final TalonFX Motorshooter1;
    private final TalonFX Motorshooter2;
    private final TalonFX Motorshooter3;
    //  private final SparkMax Elevator;
    
    private final FlywheelSim shooterPhysicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), 
        DCMotor.getKrakenX60(1),
        1.0
    );
    
    private final TalonFX intakeMotor;

    private final double MAX_ARM_POWER = 1.0;
    private final double ARM_GEAR_RATIO = 100.0; 

    private final SparkMax motor45;
    private final SparkClosedLoopController motor45Controller;

    private final SparkMax motor045;
    private final SparkClosedLoopController motor045Controller;

    // 🔥 ================= HOLD SYSTEM =================
    private double holdPosition45 = 0;
    private double holdPosition045 = 0;
    private boolean holdEnabled = true;

    public Motors() {

        Motorshooter1 = new TalonFX(39, "Carnivore");
        Motorshooter2 = new TalonFX(40, "Carnivore");
        Motorshooter3 = new TalonFX(41, "Carnivore");
    //    Elevator = new SparkMax(25, MotorType.kBrushed);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = TunerConstants.mechanisms_Current_Limits;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor = new TalonFX(42, "Carnivore");

        motor45 = new SparkMax(18, MotorType.kBrushless);
        motor45Controller = motor45.getClosedLoopController();

        SparkMaxConfig config45 = new SparkMaxConfig();
        config45.closedLoop.pid(1.0, 0.05, 0.05);
        config45.closedLoop.outputRange(-MAX_ARM_POWER, MAX_ARM_POWER); 
        config45.idleMode(SparkBaseConfig.IdleMode.kBrake);
       
        config45.softLimit.forwardSoftLimit(degreesToRotations(5));
        config45.softLimit.forwardSoftLimitEnabled(true);

        config45.softLimit.reverseSoftLimit(degreesToRotations(-5));
        config45.softLimit.reverseSoftLimitEnabled(true);

        motor45.configure(config45, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor045 = new SparkMax(19, MotorType.kBrushless);
        motor045Controller = motor045.getClosedLoopController();

        SparkMaxConfig config045 = new SparkMaxConfig();
        config045.closedLoop.pid(1.0, 0.5, 0.05);
        config045.closedLoop.outputRange(-MAX_ARM_POWER, MAX_ARM_POWER);
        config045.idleMode(SparkBaseConfig.IdleMode.kBrake);

        config045.softLimit.forwardSoftLimit(degreesToRotations(10));
        config045.softLimit.forwardSoftLimitEnabled(true);

        config045.softLimit.reverseSoftLimit(degreesToRotations(-10));
        config045.softLimit.reverseSoftLimitEnabled(true);

        motor045.configure(config045, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor45.getEncoder().setPosition(0);
        motor045.getEncoder().setPosition(0);
        
        SmartDashboard.putNumber("Inicio", motor45.getEncoder().getPosition());

        // 🔥 ASEGURAR QUE ARRANQUE QUIETO
        motor45.set(0);
        motor045.set(0);
        motor45.stopMotor();
        motor045.stopMotor();

        // 🔥 GUARDAR POSICIÓN INICIAL
        holdPosition45 = motor45.getEncoder().getPosition();
        holdPosition045 = motor045.getEncoder().getPosition();
    }

    // 🔥 HOLD AUTOMÁTICO
    @Override
    public void periodic() {
        if (holdEnabled) {
            holdArm();
        }
    }

    // 🔥 MÉTODO HOLD
    public void holdArm() {
        motor45Controller.setReference(holdPosition45, ControlType.kPosition);
        motor045Controller.setReference(holdPosition045, ControlType.kPosition);
    }

    // 🔥 DESACTIVAR HOLD (cuando presionas botón)
    public void disableHold() {
        holdEnabled = false;
    }

    // 🔥 ACTIVAR HOLD (cuando sueltas botón) 
    public void enableHold() {
        holdPosition45 = motor45.getEncoder().getPosition();
        holdPosition045 = motor045.getEncoder().getPosition();
        holdEnabled = true;
    }

    @Override
    public void simulationPeriodic() {
        var sim1 = Motorshooter1.getSimState();
        var sim2 = Motorshooter2.getSimState();

        double appliedVoltage = sim1.getMotorVoltage();

        shooterPhysicsSim.setInputVoltage(appliedVoltage);
        shooterPhysicsSim.update(0.02);

        double rps = shooterPhysicsSim.getAngularVelocityRPM() / 60.0;

        if (Math.abs(appliedVoltage) < 0.01 && Math.abs(rps) < 0.05) {
            rps = 0.0;
        }
        
        sim1.setRotorVelocity(rps);
        sim2.setRotorVelocity(-rps); 

        sim1.setSupplyVoltage(12.0);
        sim2.setSupplyVoltage(12.0);
    }

    public Command shooters() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> 
            Motorshooter3.setControl(shooterRequest.withOutput(-0.60)), this
        ),

        new WaitCommand(1.5),

        new InstantCommand(() -> {
            Motorshooter1.setControl(shooterRequest.withOutput(-0.55));
            Motorshooter2.setControl(shooterRequest.withOutput(0.55));
        }, this),

        new WaitCommand(2.0), 

        new InstantCommand(() -> { 
            Motorshooter1.setControl(shooterRequest.withOutput(0));
            Motorshooter2.setControl(shooterRequest.withOutput(0));
            Motorshooter3.setControl(shooterRequest.withOutput(0));
        }, this)
    );
}

    public void stopShooter() {
        Motorshooter1.stopMotor();
        Motorshooter2.stopMotor();
        Motorshooter3.stopMotor();
    }
/* 
   public void Elevate() {
         Elevator.set(-0.3);
     }
      public void Descend() {
         Elevator.set(0.3);
     }

  //   public void stopElevator() {
   //    Elevator.stopMotor();
  //  }/* */
    public void intakers() {
        intakeMotor.setControl(intakerRequest.withOutput(0.40));
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    private double degreesToRotations(double targetArmDegrees) {
        double motorDegrees = targetArmDegrees * ARM_GEAR_RATIO;
        return motorDegrees / 360.0;
    }

    private final double MANUAL_POWER = 0.80;

    public void extendMotor45() {
        motor45.set(-MANUAL_POWER); 
    }

    public void retractMotor45() {
        motor45.set(1); 
    }
    

    public void stopMotor45() {
        motor45.stopMotor();
    }

    public void extendMotor045() {
        motor045.set(MANUAL_POWER); 
    }

    public void retractMotor045() {
        motor045.set(-1); 
    }

    public void stopMotor045() {
        motor045.stopMotor();
    }

    public void stopAll() {
        stopShooter();
        stopIntake();
       // stopElevator();
        motor45.stopMotor();
        motor045.stopMotor();
    }
}