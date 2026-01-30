package frc.robot.subsystems.shooter.ShooterSim;

import frc.robot.constants.SimulationConstants;

public class BallKinematics {
    public static double totalBallVelocity(double rpm) {
        // surface speed transfer based formula
        double velocity1 = (rpm/60)*2*(Math.PI)*SimulationConstants.ShooterConstants.flywheelRadius;
        return velocity1*SimulationConstants.ShooterConstants.Croll;
    }
    public static double getVy(double totalV) {
        return Math.sin(Math.toRadians(SimulationConstants.ShooterConstants.shooterLaunchAngle))*totalV;
    }
    public static double getVx(double totalV, double robotAngle) {
        double xz = Math.cos(Math.toRadians(SimulationConstants.ShooterConstants.shooterLaunchAngle))*totalV;
        return Math.sin(Math.toRadians(robotAngle))*xz;
    }
    public static double getVz(double totalV, double robotAngle) {
        double xz = Math.cos(Math.toRadians(SimulationConstants.ShooterConstants.shooterLaunchAngle))*totalV;
        return Math.cos(Math.toRadians(robotAngle))*xz;
    }
}
