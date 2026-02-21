package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SegundomotorSubsystem;

public class ShooterSubsystemCommand extends Command {

    private final SegundomotorSubsystem paro;

    public ShooterSubsystemCommand(SegundomotorSubsystem paro) {
        this.paro = paro;
        addRequirements(paro);
    }

    @Override
    public void initialize() {
        paro.run(); // enciende motor (100%)
    }

    @Override
    public boolean isFinished() {
        return false; // se controla con timeout desde PathPlanner
    }

    @Override
    public void end(boolean interrupted) {
        paro.stop();
    }
}
