package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

// CONFIGURE LIMELIGHT AIM //
public class LimelightAIM extends InstantCommand{
double MaxAngularRate = RobotContainer.MaxAngularRate;
double lastTx = LimelightPID.lastTx;
double txIntegral = LimelightPID.txIntegral;
   

    public double limelightAim() {
        if (!LimelightHelpers.getTV("limelight-front")) return 0;

        double tx = LimelightHelpers.getTX("limelight-front");
        double txRate = (tx - lastTx) / 0.02;
        lastTx = tx;

        txIntegral += tx * 0.02;
        txIntegral = Math.max(Math.min(txIntegral, 0.2), -0.2);

        return -(0.015 * tx + 0.0005 * txRate + 0.00034 * txIntegral) * MaxAngularRate;
    }

}
