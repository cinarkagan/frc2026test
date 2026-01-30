package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;

public interface Controller {
    Command getAutonomousCommand();
    void getInitializeFunction();
    void configureBindings();
    void simulationPeriodic();
    void periodic();
}
