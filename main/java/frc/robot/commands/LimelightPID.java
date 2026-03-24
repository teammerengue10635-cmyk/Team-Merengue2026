package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;


// CONFIGURE LIMELIGHT PID //
public class LimelightPID extends InstantCommand {
    public static double lastTx = 0.0;
    public static double txIntegral = 0.0;

    public static double lastRange = 0.0;
    public static double rangeIntegral = 0.0;
}
