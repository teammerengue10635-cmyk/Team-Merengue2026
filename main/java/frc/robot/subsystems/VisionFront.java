package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class VisionFront {
    
    /* ================= CONSTANTES ================= */
    public static final double TX_DEADBAND = 1.0;
    public static final double MAX_LL_SPEED = 5.85;
    public static final double CAMERA_HEIGHT = 0.54;
    public static final double CAMERA_PITCH_DEG = 38; 

    /* ================= VARIABLES PID ================= */
    private double lastTx = 0.0;
    private double txIntegral = 0.0;
    
    private double lastRangeError = 0.0;
    private double rangeIntegral = 0.0;

    public VisionFront() {}

    /* ================= FUNCIONES ================= */

    public double getAimRate() {
        if (!LimelightHelpers.getTV("limelight-front")) return 0;

        double tx = LimelightHelpers.getTX("limelight-front");
        double txRate = (tx - lastTx) / 0.02;
        lastTx = tx;

        txIntegral += tx * 0.02;
        txIntegral = Math.max(Math.min(txIntegral, 0.2), -0.2);

        return -(0.015 * tx + 0.0005 * txRate + 0.00034 * txIntegral) * RobotContainer.MaxAngularRate;
    }

    // AHORA RECIBE LA DISTANCIA DESEADA Y LA ALTURA DEL TAG COMO PARÁMETROS
    public double getRangeSpeed(double targetDistanceMeters, double tagHeightMeters) {
        if (!LimelightHelpers.getTV("limelight-front")) return 0;

        double tyDeg = LimelightHelpers.getTY("limelight-front") + CAMERA_PITCH_DEG;
        double tyRad = Math.toRadians(Math.max(Math.abs(tyDeg), 0.1));

        // Trigonometría para sacar la distancia real
        double distance = (tagHeightMeters - CAMERA_HEIGHT) / Math.tan(tyRad);
        double error = distance - targetDistanceMeters;

        double dError = (error - lastRangeError) / 0.02;
        lastRangeError = error;

        rangeIntegral += error * 0.02;
        rangeIntegral = Math.max(Math.min(rangeIntegral, 0.2), -0.2);

        double outputSpeed = Math.max(Math.min(
            1.5 * error + 0.09 * dError + 0.01 * rangeIntegral,
            MAX_LL_SPEED), -MAX_LL_SPEED);
            
        // NOTA: Si ves que el robot SIGUE yendo hacia atrás cuando debería ir hacia adelante,
        // cambia el "return outputSpeed;" por "return -outputSpeed;"
        return outputSpeed; 
    }
}