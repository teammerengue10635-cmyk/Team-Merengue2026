package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;


// CONFIGURE LIMELIGHT CONSTANTS //
public class LimelightConstants extends InstantCommand {
    public static final double TX_DEADBAND = 1.0;
    public static final double MAX_LL_SPEED = 5.85;

    public static final double CAMERA_HEIGHT = 0.57;
    public static final double TAG_HEIGHT = 0.83;
    public static final double TARGET_DISTANCE = 2.5;
    public static final double COLOR_DEG = 0.0;
}
