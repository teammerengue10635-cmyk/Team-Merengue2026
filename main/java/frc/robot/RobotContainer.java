package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/* ================= PATHPLANNER ================= */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

/* ================= ROBOT ================= */
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.VisionBack;
import frc.robot.subsystems.VisionFront; // <-- IMPORTACIÓN NUEVA

public class RobotContainer {
    private boolean isArmExtended = false; // Memoria para saber dónde está el brazo

    private boolean motorUp = false;
    
    // Instanciamos la nueva clase de visión
    private final VisionFront visionCameraFront = new VisionFront();
    private final VisionBack visionCameraBack = new  VisionBack();

    /* =================== VELOCIDADES =================== */
    public static final double MaxSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public static double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* =================== SWERVE =================== */
    private final SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* =================== CONTROLES =================== */
    public final static CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController mechanisms = new CommandXboxController(1);

    /* ================= SUBSYSTEMS ================= */
    public static final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final Motors testSpark = new Motors();

    /* ================= AUTON ================= */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* =================== CONSTRUCTOR =================== */
    public RobotContainer() {

        NamedCommands.registerCommand("lanzar", testSpark.shooters());
        
        NamedCommands.registerCommand("recoger",
            new RunCommand(testSpark::intakers, testSpark).withTimeout(1.0)
        );

        NamedCommands.registerCommand("desplegar",
            new RunCommand(testSpark::extendMotor45, testSpark).withTimeout(1.5)
        );

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError(
                "No se pudo cargar RobotConfig desde PathPlanner GUI",
                e.getStackTrace()
            );
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            () -> drivetrain.getState().Speeds,
            (speeds, feedforwards) -> drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
            ),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> DriverStation.getAlliance()
                    .map(a -> a == Alliance.Red)
                    .orElse(false),
            drivetrain
        );

        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, new PathPlannerAuto(autoName));
        }
        SmartDashboard.putData("Autonomous", autoChooser);

        configureBindings();
    }

    /* =================== SPEED SCALING =================== */

    private double getDriveScale() {
        if (driver.leftBumper().getAsBoolean()) return 0.3;
        if (driver.getLeftTriggerAxis() > 0.1) return 0.5;
        return 1.0;
    }

    private double getTurnScale() {
        if (driver.rightBumper().getAsBoolean()) return 0.3;
        if (driver.getRightTriggerAxis() > 0.1) return 0.5;
        return 1.0;
    }

    /* =================== GIROS 180 =================== */

    private Command rotateRight180() {
        return drivetrain.applyRequest(() -> drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(MaxAngularRate * 0.7))
            .withTimeout(0.93);
    }

    private Command rotateLeft180() {
        return drivetrain.applyRequest(() -> drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-MaxAngularRate * 0.7))
            .withTimeout(0.93);
    }

    /* ================= APRILTAG ID ================= */

    private int getAprilTagIDFront() {
        if (!LimelightHelpers.getTV("limelight-front")) return -1;
        return (int) LimelightHelpers.getFiducialID("limelight-front");

     
    }
private int getAprilTagIDBack() {
       if (!LimelightHelpers.getTV("limelight-back")) return -1;
        return (int) LimelightHelpers.getFiducialID("limelight-back");
    }
    /* =================== BINDINGS =================== */

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * getDriveScale())
                     .withVelocityY(-driver.getLeftX() * MaxSpeed * getDriveScale())
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate * getTurnScale())
            )
        );

       driver.a().whileTrue(
            drivetrain.applyRequest(() -> {

                int tagID = getAprilTagIDFront();

                /* ===== MEDIA LUNA ===== */
                if (tagID == 9  || tagID == 10) {
                    // X = Automático a 2.3 metros (asumiendo que la altura del tag es 0.83m)
                    // Y = Controlado por el joystick izquierdo del piloto (para poder "orbitar")
                    // Rotación = Automático apuntando al Tag
                    return drive.withVelocityX(visionCameraFront.getRangeSpeed(0.7, 1.10))
                                .withVelocityY(-driver.getLeftX() * MaxSpeed * getDriveScale()) 
                                .withRotationalRate(visionCameraFront.getAimRate());
                }

                /* ===== OUTPOST ===== */
                if (tagID == 13 || tagID == 14) {
                    // ¡OJO AQUÍ! Cambia "ALTURA_DEL_TAG_AQUI" por la altura real en metros de los tags 13 y 14.
                    // Si no pones la altura correcta, el robot calculará mal y huirá.
                    // También puedes cambiar el "1.5" por la distancia a la que te quieras quedar.
                    return drive.withVelocityX(visionCameraBack.getRangeSpeed(1.5, 1.30)) 
                                .withVelocityY(0) // 0 si quieres que vaya en línea recta perfecto
                                .withRotationalRate(visionCameraBack.getAimRate());
                }

                /* ===== TOWER ===== */
                if (tagID == 15 || tagID == 16) {
                    // Lo mismo aquí, define tu distancia (ej. 1.0m) y la altura real de ese tag.
                    return drive.withVelocityX(visionCameraBack.getRangeSpeed(1.0, 1.43) * 0.8)
                                .withVelocityY(0)
                                .withRotationalRate(visionCameraBack.getAimRate());
                }

                // Si no ve ningún Tag de la lista, se detiene por seguridad
                return drive.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0);
            })
        );
        driver.y().whileTrue(
            drivetrain.applyRequest(() -> {

        int tagIDBack = getAprilTagIDBack();

          /* ===== MEDIA LUNA ===== */
                if (tagIDBack == 9 ) {
                    // X = Automático a 2.3 metros (asumiendo que la altura del tag es 0.83m)
                    // Y = Controlado por el joystick izquierdo del piloto (para poder "orbitar")
                    // Rotación = Automático apuntando al Tag
                    return drive.withVelocityX(visionCameraBack.getRangeSpeed(0.5, 1.10))
                                .withVelocityY(-driver.getLeftX() * MaxSpeed * getDriveScale()) 
                                .withRotationalRate(visionCameraBack.getAimRate());
                }

                /* ===== OUTPOST ===== */
                if (tagIDBack == 13 || tagIDBack == 14) {
                    // ¡OJO AQUÍ! Cambia "ALTURA_DEL_TAG_AQUI" por la altura real en metros de los tags 13 y 14.
                    // Si no pones la altura correcta, el robot calculará mal y huirá.
                    // También puedes cambiar el "1.5" por la distancia a la que te quieras quedar.
                    return drive.withVelocityX(visionCameraBack.getRangeSpeed(1.5, 1.30)) 
                                .withVelocityY(0) // 0 si quieres que vaya en línea recta perfecto
                                .withRotationalRate(visionCameraBack.getAimRate());
                }

                /* ===== TOWER ===== */
                if (tagIDBack == 15 || tagIDBack == 16) {
                    // Lo mismo aquí, define tu distancia (ej. 1.0m) y la altura real de ese tag.
                    return drive.withVelocityX(visionCameraBack.getRangeSpeed(1.0, 1.43) * 0.8)
                                .withVelocityY(0)
                                .withRotationalRate(visionCameraBack.getAimRate());
                }

                // Si no ve ningún Tag de la lista, se detiene por seguridad
                return drive.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0);
            })
        );

        driver.b().onTrue(rotateRight180());
        driver.x().onTrue(rotateLeft180());

        mechanisms.leftTrigger()
            .whileTrue(new RunCommand(testSpark::intakers, testSpark))
            .onFalse(new InstantCommand(testSpark::stopIntake, testSpark));

            mechanisms.rightTrigger()
            .whileTrue(testSpark.shooters())
            .onFalse(new InstantCommand(testSpark::stopShooter, testSpark));

/*         // Botón X: RECALIBRAR (Pone los sensores a 0 en la posición actual)
        mechanisms.x().onTrue(
            Commands.runOnce(() -> {
                testSpark.resetArmEncoders();
                isArmExtended = false; // Actualizamos la memoria porque el brazo ahora está guardado
            }, testSpark)
        );

        // Botón Y: ALTERNAR (Si está guardado lo saca, si está afuera lo guarda)
        mechanisms.y().onTrue(
            Commands.runOnce(() -> {
                if (!isArmExtended) {
                    // Si NO está extendido, lo sacamos
                    testSpark.extendMotor45();
                    testSpark.extendMotor045();
                    isArmExtended = true;
                } else {
                    // Si YA está extendido, lo guardamos
                    testSpark.retractMotor45();
                    testSpark.retractMotor045();
                    isArmExtended = false;
                }
            }, testSpark)
        ); */

             // Botón a: MIENTRAS se mantenga presionado, el brazo se EXTIENDE. Al soltar, se apaga.
        mechanisms.a().whileTrue(
            Commands.startEnd(
                () -> {
                    testSpark.disableHold(); 
                    testSpark.extendMotor45();
                    testSpark.extendMotor045();
                },
                () -> {
                    testSpark.stopMotor45();
                    testSpark.stopMotor045();
                    testSpark.enableHold(); 
                },
                testSpark
            )
        );

        mechanisms.b().whileTrue(
            Commands.startEnd(
                () -> {
                    testSpark.disableHold(); 
                    testSpark.retractMotor45();
                    testSpark.retractMotor045();
                },
                () -> {
                    testSpark.stopMotor45();
                    testSpark.stopMotor045();
                    testSpark.enableHold(); 
                },
                testSpark
            )
        );



        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /* ================= AUTON ================= */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command intakeOn() { return new InstantCommand(testSpark::intakers, testSpark); }
    private Command intakeOff() { return new InstantCommand(testSpark::stopIntake, testSpark); }
    private Command shooterOn() { return new InstantCommand(testSpark::shooters, testSpark); }
    private Command shooterOff() { return new InstantCommand(testSpark::stopShooter, testSpark); }
    private Command extend() { return new InstantCommand(testSpark::extendMotor45, testSpark); }
    private Command retract() { return new InstantCommand(testSpark::retractMotor45, testSpark); }

    public Object getMotors() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotors'");
    }

}