package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SegundomotorSubsystem;

public class ShooterSubsystemCommand extends Command {

    private final SegundomotorSubsystem paro;

    public ShooterSubsystemCommand(SegundomotorSubsystem paro) {
        this.paro = paro;
        addRequirements(paro);
    }
    // MOTORS ON (100%)
    @Override
    public void initialize() {
        paro.run(); 
    }
    
    // CONTROL TIMEOUT WHIT PATHPLANER
    @Override
    public boolean isFinished() {
        return false; 
    }
    
    // MOTORS ON (100%)
    @Override
    public void end(boolean interrupted) {
        paro.stop();
    }
}
