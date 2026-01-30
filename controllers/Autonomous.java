package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.MachineSubsystem;
import frc.robot.Telemetry;
import frc.robot.constants.TeleopConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSimulation;
import frc.robot.utils.Container;

public class Autonomous implements Controller {
    private boolean simulationMode = Container.simulationMode;
    private Telemetry logger;
    private ShooterSimulation shooterSimulation;
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TeleopConstants.MaxSpeed * 0.1).withRotationalDeadband(TeleopConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private MachineSubsystem machineSubsystem;
    public Autonomous(Telemetry logger,CommandSwerveDrivetrain drivetrain, MachineSubsystem machineSubsystem) {
        this.logger = logger;
        this.drivetrain = drivetrain;
        //this.joystick = new Joystick(0);
        this.shooterSimulation = new ShooterSimulation(this.drivetrain);
        this.machineSubsystem = machineSubsystem;
    }
    @Override
    public void getInitializeFunction() {}

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }

    @Override
    public void simulationPeriodic() {} 

    @Override
    public void periodic() {}
    
}
