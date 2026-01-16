package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConstants {
    public static CANBus canbus = new CANBus("rio");

    public static int shooter1_ID = 61;
    public static int shooter2_ID = 24;
    public static int feeder_ID = 25;

    public static double feederRPM = 2000;
}
