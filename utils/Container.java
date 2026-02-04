package frc.robot.utils;

public class Container {
    public static boolean driveEnabled = true;
    public static boolean simulationMode = true;
    public static double fuelCount = 0;
    public static void increaseFuel() {
        fuelCount++;
    }
}
