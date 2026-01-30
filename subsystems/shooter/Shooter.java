package frc.robot.subsystems.shooter;

import frc.robot.utils.AllStates;

public interface Shooter {
    public void requestState(AllStates.ShooterStates state); 
    public void customRequest(double rpm);
    public boolean isAtRPM();
    public double getRPM();
    public double getRPMHood();
    public double getRPMFlywheel();
}
