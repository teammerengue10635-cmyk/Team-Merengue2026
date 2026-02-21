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
import frc.robot.subsystems.SegundomotorSubsystem;
import frc.robot.subsystems.Motors;

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
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController mechanisms = new CommandXboxController(1);

    /* ================= SUBSYSTEMS ================= */
    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final Motors testSpark = new Motors();
    private final SegundomotorSubsystem shooter = new SegundomotorSubsystem();

    /* ================= AUTON ================= */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* =================== CONSTRUCTOR =================== */
    public RobotContainer() {

        NamedCommands.registerCommand("recoger", intakeOn());
        NamedCommands.registerCommand("apagar recogedor", intakeOff());
        NamedCommands.registerCommand("lanzar", shooterOn());
        NamedCommands.registerCommand("apagar lanzador", shooterOff());

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
        if (driver.leftBumper().getAsBoolean()) return 0.5;
        if (driver.getLeftTriggerAxis() > 0.1) return 0.75;
        return 1.0;
    }

    private double getTurnScale() {
        if (driver.rightBumper().getAsBoolean()) return 0.5;
        if (driver.getRightTriggerAxis() > 0.1) return 0.75;
        return 1.0;
    }

    /* ================= LIMELIGHT CONSTANTS ================= */
    private static final double TX_DEADBAND = 1.0;
    private static final double MAX_LL_SPEED = 4.58;

    private static final double CAMERA_HEIGHT = 0.36;
    private static final double TAG_HEIGHT = 0.83;
    private static final double TARGET_DISTANCE = 2.5;
    private static final double CAMERA_PITCH_DEG = 0.0;

    /* ================= PID AIM ================= */
    private double limelightAim() {
        if (!LimelightHelpers.getTV("limelight")) return 0;

        double tx = LimelightHelpers.getTX("limelight");
        double txRate = (tx - lastTx) / 0.02;
        lastTx = tx;

        txIntegral += tx * 0.02;
        txIntegral = Math.max(Math.min(txIntegral, 0.2), -0.2);

        return -(0.015 * tx + 0.0005 * txRate + 0.00034 * txIntegral) * MaxAngularRate;
    }

    /* ================= PID RANGE ================= */
    private double limelightRange() {
        if (!LimelightHelpers.getTV("limelight")) return 0;

        double tyDeg = LimelightHelpers.getTY("limelight") + CAMERA_PITCH_DEG;
        double tyRad = Math.toRadians(Math.max(Math.abs(tyDeg), 0.1));

        double distance = (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(tyRad);
        double error = distance - TARGET_DISTANCE;

        double dError = (error - lastRange) / 0.02;
        lastRange = error;

        rangeIntegral += error * 0.02;
        rangeIntegral = Math.max(Math.min(rangeIntegral, 0.2), -0.2);

        return Math.max(Math.min(
            1.5 * error + 0.09 * dError + 0.01 * rangeIntegral,
            MAX_LL_SPEED), -MAX_LL_SPEED);
    }

    /* =================== GIROS 180 =================== */ 
    private Command rotateRight180() 
      { return drivetrain.applyRequest(() -> drive.withVelocityX(0) .withVelocityY(0) 
        .withRotationalRate(MaxAngularRate * 0.7) ).withTimeout(0.93); } 

    
    private Command  rotateLeft180()
      { return drivetrain.applyRequest(() -> drive.withVelocityX(0) .withVelocityY(0)
        .withRotationalRate(-MaxAngularRate * 0.7) ).withTimeout(0.93); }

    /* ================= APRILTAG ID ================= */
    private int getAprilTagID() {
        if (!LimelightHelpers.getTV("limelight")) return -1;
        return (int) LimelightHelpers.getFiducialID("limelight");
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

                int tagID = getAprilTagID();

                if (tagID == 13) {
                    return drive.withVelocityX(limelightRange())
                                .withVelocityY(0)
                                .withRotationalRate(limelightAim());
                }

                if (tagID == 7) {
                    return drive.withRotationalRate(MaxAngularRate * 0.7);
                }

                if (tagID == 6) {
                    return drive.withRotationalRate(-MaxAngularRate * 0.7);
                }

                if (tagID == 14) {
                    return drive.withVelocityY(MaxSpeed * 0.5);
                }

                if (tagID == 15) {
                    return drive.withVelocityY(-MaxSpeed * 0.5);
                }

                return drive.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0);
            })
        );
        
        driver.b().onTrue(rotateRight180());
        driver.x().onTrue(rotateLeft180());

        mechanisms.x()
            .whileTrue(new RunCommand(testSpark::intakers, testSpark))
            .onFalse(new InstantCommand(testSpark::stopIntake, testSpark));

        mechanisms.y()
            .whileTrue(new RunCommand(testSpark::shooters, testSpark))
            .onFalse(new InstantCommand(testSpark::stopShooter, testSpark));

        mechanisms.a()
            .onTrue(new InstantCommand(testSpark::moveTo45Left, testSpark))
            .onTrue(new InstantCommand(testSpark::moveTo045Left, testSpark));

        mechanisms.b()
            .onTrue(new InstantCommand(testSpark::moveTo45Right, testSpark))
            .onTrue(new InstantCommand(testSpark::moveTo045Right, testSpark));

       

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
}