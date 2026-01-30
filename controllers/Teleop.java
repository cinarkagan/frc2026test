package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.MachineSubsystem;
import frc.robot.Telemetry;
import frc.robot.commands.TurnToAngle;
import frc.robot.constants.TeleopConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSimulation;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllStates;
import frc.robot.utils.Container;

public class Teleop implements Controller {
    private boolean driveEnabled = Container.driveEnabled;
    private boolean simulationMode = Container.simulationMode;
    private Telemetry logger;
    private ShooterSimulation shooterSimulation;
    //private final SlewRateLimiter limiter = new SlewRateLimiter(0.8);
    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController joystick;
    //private Joystick joystick;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TeleopConstants.MaxSpeed * 0.1).withRotationalDeadband(TeleopConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private MachineSubsystem machineSubsystem;
    public Teleop(Telemetry logger,CommandSwerveDrivetrain drivetrain, MachineSubsystem machineSubsystem) {
        this.logger = logger;
        this.drivetrain = drivetrain;
        //this.joystick = new Joystick(0);
        this.joystick = new CommandXboxController(0);
        this.shooterSimulation = new ShooterSimulation(this.drivetrain);
        this.machineSubsystem = machineSubsystem;
    }
    @Override
    public void getInitializeFunction() {
        configureBindings();
    }
    public ShooterSimulation getShooterSimulation() {
        return shooterSimulation;
    }
    public void configureBindings() {
        // TODO Auto-generated method stub
        if (simulationMode) {
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            if (driveEnabled) {
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(joystick.getLeftY() * TeleopConstants.MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(joystick.getLeftX() * TeleopConstants.MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(0)//-joystick.getRightX() * TeleopConstants.MaxAngularRate)// Drive counterclockwise with negative X (left)
                    )
                );
            }
            /*if (driveEnabled) {
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(joystick.getRawAxis(1) * TeleopConstants.MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(joystick.getRawAxis(0) * TeleopConstants.MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(0)//-joystick.getRawAxis(2)-(0.1)) * TeleopConstants.MaxAngularRate)// Drive counterclockwise with negative X (left)
                    )
                );
            }*/
            // Idle while the robot is disabled. This ensures the conf  igured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
            );

            //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
            //joystick.b().whileTrue(new TurnToAngle(180, drivetrain, drive,MaxAngularRate));
            //joystick.a().whileTrue(new TurnToAngle(66, drivetrain, drive,TeleopConstants.MaxAngularRate));
            //joystick.b().onTrue(shooterSubsystem.disableShooter());

            /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
            ));*/

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.st art().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

            // Reset the field-centric heading on left bumper press.
            //joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
            joystick.leftBumper().onTrue(new InstantCommand(()->{shooterSimulation.launchFuel();}));
            joystick.rightBumper().onTrue(new InstantCommand(()->{shooterSimulation.requestState(AllStates.ShooterStates.P1_SHOOT);}));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    /*public Joystick getJoystick() {
        return joystick;
    }*/
    public void simulationPeriodic() {}
    public void periodic() {}
}
