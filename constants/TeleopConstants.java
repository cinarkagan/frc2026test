package frc.robot.constants;

import frc.robot.utils.Container;
import static edu.wpi.first.units.Units.*;

public class TeleopConstants {
    public static double MaxSpeed = 0.75 * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static  double MaxAngularRate = RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
}
