package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;

public interface Controller {
    void getInitializeFunction();
    void simulationPeriodic();
    void periodic();
}
