package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestSparkSubsystem;


public class RobotContainer {

    /* =================== LIMELIGHT PID STATE =================== */
    private double lastTx = 0.0;
    private double txIntegral = 0.0;

    private double lastRange = 0.0;
    private double rangeIntegral = 0.0;

    /* =================== VELOCIDADES =================== */
    private final double MaxSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* =================== SWERVE =================== */
    private final SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* =================== CONTROLES =================== */
    private final CommandXboxController driver =
        new CommandXboxController(0);   // MANEJO

    private final CommandXboxController mechanisms =
        new CommandXboxController(1);   // MECANISMOS

    /* =================== SUBSYSTEMS =================== */
    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final TestSparkSubsystem testSpark =
        new TestSparkSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    /* =================== LIMELIGHT CONSTANTS =================== */
    private static final double TX_DEADBAND = 1.0;
    private static final double MAX_LL_SPEED = 4.58;

    private static final double CAMERA_HEIGHT = 0.36; // altura del piso a la camara
    private static final double TAG_HEIGHT = 0.83; // altura del piso al centro del AprilTag
    private static final double TARGET_DISTANCE = 2.5;
    private static final double CAMERA_PITCH_DEG = 0.0;

    /* =================== AIM (PID) =================== */
    private double limelightAim() {
        if (!LimelightHelpers.getTV("limelight")) {
            lastTx = 0.0;
            txIntegral = 0.0;
            return 0.0;
        }

        double tx = LimelightHelpers.getTX("limelight");

        if (Math.abs(tx) < TX_DEADBAND) {
            lastTx = tx;
            txIntegral = 0.0;
            return 0.0;
        }

        double txRate = (tx - lastTx) / 0.02;
        lastTx = tx;

        txIntegral += tx * 0.02;
        txIntegral = Math.max(Math.min(txIntegral, 0.2), -0.2);

        double kP = 0.015;
        double kD = 0.0005;
        double kI = 0.00034;

        return -(kP * tx + kD * txRate + kI * txIntegral) * MaxAngularRate;
    }

    /* =================== RANGE (PID) =================== */
    private double limelightRange() {
        if (!LimelightHelpers.getTV("limelight")) return 0.0;

        double tyDeg = LimelightHelpers.getTY("limelight") + CAMERA_PITCH_DEG;
        double tyRad = Math.toRadians(Math.max(Math.abs(tyDeg), 0.1));

        double distance = (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(tyRad);
        double error = distance - TARGET_DISTANCE;

        if (Math.abs(error) < 0.02) {
            lastRange = 0.0;
            rangeIntegral = 0.0;
            return 0.0;
        }

        double dError = (error - lastRange) / 0.02;
        lastRange = error;

        rangeIntegral += error * 0.02;
        rangeIntegral = Math.max(Math.min(rangeIntegral, 0.2), -0.2);

        double rkP = 1.5;
        double rkD = 0.09;
        double rkI = 0.01;

        return Math.max(Math.min(
            rkP * error + rkD * dError + rkI * rangeIntegral,
            MAX_LL_SPEED), -MAX_LL_SPEED);
    }

    /* =================== GIROS 180° =================== */
    private Command rotateRight180() {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(MaxAngularRate * 0.7)
        ).withTimeout(0.93);
    }

    private Command rotateLeft180() {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(-MaxAngularRate * 0.7)
        ).withTimeout(0.93);
    }

    /* =================== BINDINGS =================== */
    private void configureBindings() {

        /* ===== MANEJO NORMAL ===== */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        /* ===== AUTO AIM ===== */
        driver.a().whileTrue(
            drivetrain.applyRequest(() -> {
                double vx = limelightRange();
                double rot = limelightAim();

                return drive
                    .withVelocityX(vx)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(rot - driver.getRightX() * MaxAngularRate);
            })
        );

        /* ===== GIROS ===== */
        driver.b().onTrue(rotateRight180());
        driver.x().onTrue(rotateLeft180());

        /* ===== MECANISMOS (OPERATOR) ===== */
        mechanisms.leftTrigger()
            .whileTrue(new RunCommand(testSpark::shooters, testSpark))
            .onFalse(new InstantCommand(testSpark::stop0, testSpark));

        mechanisms.rightTrigger()
            .whileTrue(new RunCommand(testSpark::intakers, testSpark))
            .onFalse(new InstantCommand(testSpark::stop0, testSpark));

        /* ===== DISABLED ===== */
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /* =================== AUTON =================== */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            ).withTimeout(5),
            drivetrain.applyRequest(() -> new SwerveRequest.Idle())
        );
    }
}
