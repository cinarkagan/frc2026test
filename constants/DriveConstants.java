package frc.robot.constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveConstants {
    public static double driveP = 0;
    public static double driveI = 0;
    public static double driveD = 0;
    public static double turnP = 0.0314;
    public static double turnI = 0;
    public static double turnD = 0.0108;
    public static double turnToleranceDeg = 3;
    public static double turnToleranceDegPerSec = 5;

      private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
    private  DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    
}
