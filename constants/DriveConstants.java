package frc.robot.constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveConstants {
    public static double driveP = 10;
    public static double driveI = 0;
    public static double driveD = 0.054;
    public static double turnP = 8;
    public static double turnI = 0;
    public static double turnD = 0;
    public static double turnToleranceDeg = 3;
    public static double turnToleranceDegPerSec = 5;
    public static double toleranceYCM = 5;
    public static double toleranceXCM = 5;
    public static double goalX = 25;
    public static double goalY = 25;
}
