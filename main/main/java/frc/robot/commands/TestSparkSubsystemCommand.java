package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motors;

public class TestSparkSubsystemCommand extends Command {

    private final Motors shooter;

    public TestSparkSubsystemCommand(Motors shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.intakers(); // enciende motor (100%)
    }

    @Override
    public boolean isFinished() {
        return false; // se controla con timeout desde PathPlanner
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntake();
    }
}
