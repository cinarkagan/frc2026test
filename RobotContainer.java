// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.controller.Controller;
import frc.robot.controller.ControllerManager;
import frc.robot.controller.GamepadController;
import frc.robot.controller.AutoController;
import frc.robot.robot.RobotCore;
import frc.robot.robot.constants.SwerveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.robot.subsystems.ShooterSubsystem;
import frc.robot.robot.subsystems.LocalizationSubsystem;

public class RobotContainer {
    // Speed limits
    private final double maxSpeed = 0.75 * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.6).in(RadiansPerSecond);

    // Hardware
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(drivetrain);

    // Core robot state/action handler
    public final RobotCore robotCore;

    // Controllers
    public final GamepadController gamepadController;
    public final AutoController autoController;
    public final ControllerManager controllerManager;

    // Telemetry
    private final Telemetry logger = new Telemetry(maxSpeed);

    public RobotContainer() {
        // Initialize RobotCore
        robotCore = new RobotCore(
            drivetrain,
            shooterSubsystem,
            localizationSubsystem,
            maxSpeed,
            maxAngularRate
        );

        // Initialize controllers
        gamepadController = new GamepadController(joystick, maxSpeed, maxAngularRate);
        autoController = new AutoController(drivetrain);

        // Initialize controller manager with gamepad as default
        controllerManager = new ControllerManager(gamepadController);

        // Configure basic bindings
        configureBindings();

        // Register telemetry
        drivetrain.registerTelemetry(logger::telemeterize);

        // Configure PathPlanner (when vendordeps is added)
        drivetrain.configurePathPlanner();
    }

    private void configureBindings() {
        // Idle while disabled to apply configured neutral mode
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    /**
     * Get the active controller (used by Robot.java periodic methods).
     */
    public Controller getActiveController() {
        return controllerManager.getActiveController();
    }

    /**
     * Prepare autonomous mode.
     */
    public void prepareAutonomous() {
        // Switch to auto controller - it will use the SendableChooser selection
        controllerManager.setActiveController(autoController);
    }

    /**
     * Prepare teleop mode.
     */
    public void prepareTeleop() {
        controllerManager.setActiveController(gamepadController);
    }

    /**
     * Get the autonomous command from the auto controller.
     * This can be used by Robot.java if using standard WPILib autonomous.
     */
    public Command getAutonomousCommand() {
        return autoController.getSelectedCommand();
    }

    // Getters for subsystems
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public LocalizationSubsystem getLocalizationSubsystem() {
        return localizationSubsystem;
    }
}
