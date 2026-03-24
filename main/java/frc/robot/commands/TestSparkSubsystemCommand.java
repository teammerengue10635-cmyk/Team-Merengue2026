package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motors;

public class TestSparkSubsystemCommand extends Command {

    private final Motors shooter;

    public TestSparkSubsystemCommand(Motors shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    // MOTORS ON (100%)
    @Override
    public void initialize() {
        shooter.intakers();
    }

    // CONTROL TIMEOUT WHIT PATHPLANER
    @Override
    public boolean isFinished() {
        return false; 
    }

    // MOTORS OFF 
    @Override
    public void end(boolean interrupted) {
        shooter.stopIntake();
    }
}
