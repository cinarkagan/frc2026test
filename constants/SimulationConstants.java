package frc.robot.constants;

public class SimulationConstants {
    public class ShooterConstants {
        public static double ShooterHeight = 0.2; //Xsh
        public static double g = 9.81;
        public static double Croll = 1; //0.5 if one side shooter/one wheel shooter (fuel rotation coef)
        public static double flywheelRadius = 2 * (2.54/100); //2.54/100 is unit conversion, the 2 is in inches, rw
        public static double shooterLaunchAngle = 35;  //Ob

    }
}
