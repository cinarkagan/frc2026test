package frc.robot.controller.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.constants.DriveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that drives the robot to a target pose using PID control.
 * Uses the same PID values as DriveToTag and TurnToAngle commands.
 */
public class DriveToPoseCommand extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    
    private final SwerveRequest.FieldCentric driveRequest;
    private final double maxSpeed;
    private final double maxAngularSpeed;

    /**
     * Creates a new DriveToPoseCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param targetPose The target pose to drive to
     * @param maxSpeed Maximum linear speed in m/s
     * @param maxAngularSpeed Maximum angular speed in rad/s
     */
    public DriveToPoseCommand(
            CommandSwerveDrivetrain drivetrain,
            Pose2d targetPose,
            double maxSpeed,
            double maxAngularSpeed) {
        
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;

        // Use existing PID values from DriveConstants
        this.xController = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        this.yController = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        this.thetaController = new PIDController(
            DriveConstants.turnP,
            DriveConstants.turnI,
            DriveConstants.turnD
        );

        // Configure controllers
        xController.setTolerance(AutoConstants.POSITION_TOLERANCE);
        yController.setTolerance(AutoConstants.POSITION_TOLERANCE);
        thetaController.setTolerance(Math.toRadians(AutoConstants.ANGLE_TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create drive request
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.02)
            .withRotationalDeadband(maxAngularSpeed * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    /**
     * Creates a DriveToPoseCommand with default speed limits.
     */
    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this(drivetrain, targetPose, 
             AutoConstants.MAX_VELOCITY, 
             AutoConstants.MAX_ANGULAR_VELOCITY);
    }

    @Override
    public void initialize() {
        // Reset controllers and set targets
        xController.reset();
        yController.reset();
        thetaController.reset();
        
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        // Calculate PID outputs
        double xOutput = xController.calculate(currentPose.getX());
        double yOutput = yController.calculate(currentPose.getY());
        double thetaOutput = thetaController.calculate(
            currentPose.getRotation().getRadians()
        );

        // Clamp to max speeds
        xOutput = MathUtil.clamp(xOutput, -maxSpeed, maxSpeed);
        yOutput = MathUtil.clamp(yOutput, -maxSpeed, maxSpeed);
        thetaOutput = MathUtil.clamp(thetaOutput, -maxAngularSpeed, maxAngularSpeed);

        // Apply drive request
        drivetrain.setControl(driveRequest
            .withVelocityX(xOutput)
            .withVelocityY(yOutput)
            .withRotationalRate(thetaOutput));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() 
            && yController.atSetpoint() 
            && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
