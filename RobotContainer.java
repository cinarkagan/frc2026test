// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.TurnToAngle;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TeleopConstants;
import frc.robot.controllers.Teleop;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Container;
import frc.robot.utils.FuelSim;

public class RobotContainer {
    private boolean simulationMode = Container.simulationMode;
    private final Telemetry logger = new Telemetry(TeleopConstants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    //private final SlewRateLimiter limiter = new SlewRateLimiter(0.8);
    public final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(drivetrain);
    public Teleop teleopController = new Teleop(logger, drivetrain, shooterSubsystem, joystick);
    
    public RobotContainer() {
        teleopController.getInitializeFunction();
        configureFuelSim();
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }
    private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
            (0.6),
            (0.6),
            (0.2),
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Speeds);

    instance.registerIntake(
            -(0.3),
            (0.3),
            (0.3),
            (0.4),
            () -> true,
            Container::increaseFuel);

    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
            
FuelSim.getInstance().start(); 
}
}
