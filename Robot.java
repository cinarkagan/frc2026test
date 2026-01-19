// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.robot.state.RobotState;
import frc.robot.robot.state.RobotAction;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    // Log and replay timestamp and joystick data
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        // Still need CommandScheduler for subsystem periodic methods
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.prepareAutonomous();
    }

    @Override
    public void autonomousPeriodic() {
        runControllerLoop();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        m_robotContainer.prepareTeleop();
    }

    @Override
    public void teleopPeriodic() {
        runControllerLoop();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Core controller loop - runs each periodic in teleop and auto.
     * 1. Build robot state from subsystems
     * 2. Pass state to active controller
     * 3. Get action from controller
     * 4. Execute action on robot
     */
    private void runControllerLoop() {
        // 1. Build current robot state
        RobotState state = m_robotContainer.robotCore.buildState();

        // 2. Get action from active controller
        RobotAction action = m_robotContainer.getActiveController().update(state);

        // 3. Execute the action
        m_robotContainer.robotCore.executeAction(action);
    }
}
