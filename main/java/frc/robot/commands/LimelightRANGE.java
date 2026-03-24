package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;

// CONFIGURE LIMELIGHT RANGE //
public class LimelightRANGE extends InstantCommand{
    double TX_DEADBAND = LimelightConstants.TX_DEADBAND;
    double MAX_LL_SPEED = LimelightConstants.MAX_LL_SPEED;

    double CAMERA_HEIGHT = LimelightConstants.CAMERA_HEIGHT;
    double TAG_HEIGHT = LimelightConstants.TAG_HEIGHT;
    double TARGET_DISTANCE = LimelightConstants.TARGET_DISTANCE;
    double CAMERA_PITCH_DEG = LimelightConstants.COLOR_DEG;
    double lastRange = LimelightPID.lastRange;
    double rangeIntegral = LimelightPID.rangeIntegral;

public double LimelightRange()
{
        if (!LimelightHelpers.getTV("limelight-front")) return 0;

        double tyDeg = LimelightHelpers.getTY("limelight-front") + CAMERA_PITCH_DEG;
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
  } 