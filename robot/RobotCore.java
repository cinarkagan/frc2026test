package frc.robot.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.LimelightHelpers;
import frc.robot.robot.state.RobotState;
import frc.robot.robot.state.RobotAction;
import frc.robot.robot.state.RobotAction.ActionType;
import frc.robot.robot.constants.DriveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.robot.subsystems.ShooterSubsystem;
import frc.robot.robot.subsystems.LocalizationSubsystem;

/**
 * Central hub that:
 * 1. Aggregates robot state from all subsystems into RobotState
 * 2. Executes RobotAction by translating to subsystem commands
 *
 * This class bridges the controller layer and the subsystem layer.
 */
public class RobotCore {

    // Subsystem references
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final LocalizationSubsystem localization;

    // Swerve requests for direct control
    private final SwerveRequest.FieldCentric fieldCentricRequest;
    private final SwerveRequest.RobotCentric robotCentricRequest;
    private final SwerveRequest.Idle idleRequest;

    // Maximum speeds
    private final double maxSpeed;
    private final double maxAngularRate;

    // PID controllers for navigation
    private final PIDController navControllerX;
    private final PIDController navControllerY;
    private final PIDController turnController;

    // Navigation state
    private boolean isNavigating = false;
    private boolean isTurning = false;
    private double navTargetX = 0;
    private double navTargetY = 0;
    private double turnTargetAngle = 0;

    public RobotCore(CommandSwerveDrivetrain drivetrain,
                     ShooterSubsystem shooter,
                     LocalizationSubsystem localization,
                     double maxSpeed, double maxAngularRate) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.localization = localization;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        // Initialize swerve requests
        this.fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.idleRequest = new SwerveRequest.Idle();

        // Navigation PID controllers
        this.navControllerX = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        this.navControllerY = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        this.turnController = new PIDController(
            DriveConstants.turnP,
            DriveConstants.turnI,
            DriveConstants.turnD
        );

        // Configure tolerances
        navControllerX.setTolerance(0.15);
        navControllerY.setTolerance(0.15);
        turnController.setTolerance(Math.toRadians(DriveConstants.turnToleranceDeg));
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Build a RobotState snapshot from all subsystems.
     */
    public RobotState buildState() {
        var driveState = drivetrain.getState();
        Pose2d pose = driveState.Pose;
        var speeds = driveState.Speeds;

        var visionEstimate = localization.getLimelightPoseEstimate();
        boolean hasVision = localization.hasAnEstimate();

        return new RobotState.Builder()
            .positionX(pose.getX())
            .positionY(pose.getY())
            .headingDegrees(drivetrain.getGyroHeading())
            .velocityX(speeds.vxMetersPerSecond)
            .velocityY(speeds.vyMetersPerSecond)
            .angularVelocity(speeds.omegaRadiansPerSecond)
            .shooterTargetRPM1(shooter.goalRPM)
            .shooterTargetRPM2(shooter.goalRPM2)
            .shooterEnabled(shooter.enabled)
            .hasVisionEstimate(hasVision)
            .visibleTagCount(hasVision ? visionEstimate.tagCount : 0)
            .visionPose(hasVision ? visionEstimate.pose : new Pose2d())
            .timestamp(Timer.getFPGATimestamp())
            .build();
    }

    /**
     * Execute a RobotAction by translating to appropriate subsystem commands.
     */
    public void executeAction(RobotAction action) {
        switch (action.getType()) {
            case NONE:
                isNavigating = false;
                isTurning = false;
                drivetrain.setControl(idleRequest);
                break;

            case DRIVE:
                isNavigating = false;
                isTurning = false;
                executeDriveAction(action);
                break;

            case NAVIGATE:
                isTurning = false;
                executeNavigateAction(action);
                break;

            case TURN:
                isNavigating = false;
                executeTurnAction(action);
                break;

            case SHOOTER:
                executeShooterAction(action);
                break;

            case COMPOSITE:
                executeCompositeAction(action);
                break;
        }
    }

    private void executeDriveAction(RobotAction action) {
        action.getDriveAction().ifPresent(drive -> {
            if (drive.fieldRelative) {
                drivetrain.setControl(fieldCentricRequest
                    .withVelocityX(drive.velocityX)
                    .withVelocityY(drive.velocityY)
                    .withRotationalRate(drive.rotationRate));
            } else {
                drivetrain.setControl(robotCentricRequest
                    .withVelocityX(drive.velocityX)
                    .withVelocityY(drive.velocityY)
                    .withRotationalRate(drive.rotationRate));
            }
        });
    }

    private void executeNavigateAction(RobotAction action) {
        action.getNavigateAction().ifPresent(nav -> {
            // Set or update target
            if (!isNavigating || navTargetX != nav.targetX || navTargetY != nav.targetY) {
                navTargetX = nav.targetX;
                navTargetY = nav.targetY;
                navControllerX.setSetpoint(nav.targetX);
                navControllerY.setSetpoint(nav.targetY);
                isNavigating = true;
            }

            // Get current pose
            var state = drivetrain.getState();
            double currentX = state.Pose.getX();
            double currentY = state.Pose.getY();

            // Calculate PID outputs
            double outputX = navControllerX.calculate(currentX);
            double outputY = navControllerY.calculate(currentY);

            // Clamp outputs
            outputX = MathUtil.clamp(outputX, -maxSpeed * 0.5, maxSpeed * 0.5);
            outputY = MathUtil.clamp(outputY, -maxSpeed * 0.5, maxSpeed * 0.5);

            // Handle optional heading
            double rotationRate = 0;
            if (nav.targetHeading != null) {
                double currentHeading = drivetrain.getGyroHeading();
                turnController.setSetpoint(Math.toRadians(nav.targetHeading));
                rotationRate = turnController.calculate(Math.toRadians(currentHeading));
                rotationRate = MathUtil.clamp(rotationRate, -maxAngularRate, maxAngularRate);
            }

            drivetrain.setControl(fieldCentricRequest
                .withVelocityX(outputX)
                .withVelocityY(outputY)
                .withRotationalRate(rotationRate));
        });
    }

    private void executeTurnAction(RobotAction action) {
        action.getTurnAction().ifPresent(turn -> {
            // Set or update target
            if (!isTurning || turnTargetAngle != turn.targetAngleDegrees) {
                turnTargetAngle = turn.targetAngleDegrees;
                turnController.setSetpoint(Math.toRadians(turn.targetAngleDegrees));
                isTurning = true;
            }

            // Calculate output
            double currentHeading = Math.toRadians(drivetrain.getGyroHeading());
            double output = turnController.calculate(currentHeading);
            output = MathUtil.clamp(output, -maxAngularRate, maxAngularRate);

            drivetrain.setControl(fieldCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(output));
        });
    }

    private void executeShooterAction(RobotAction action) {
        action.getShooterAction().ifPresent(shooterAction -> {
            switch (shooterAction.command) {
                case ENABLE:
                    shooter.enabled = true;
                    break;
                case DISABLE:
                    shooter.enabled = false;
                    break;
                case ADJUST_RPM:
                    shooter.goalRPM += shooterAction.rpmDelta;
                    break;
                case NONE:
                    break;
            }
        });
    }

    private void executeCompositeAction(RobotAction action) {
        // Execute drive
        action.getDriveAction().ifPresent(drive -> {
            if (drive.fieldRelative) {
                drivetrain.setControl(fieldCentricRequest
                    .withVelocityX(drive.velocityX)
                    .withVelocityY(drive.velocityY)
                    .withRotationalRate(drive.rotationRate));
            } else {
                drivetrain.setControl(robotCentricRequest
                    .withVelocityX(drive.velocityX)
                    .withVelocityY(drive.velocityY)
                    .withRotationalRate(drive.rotationRate));
            }
        });

        // Execute shooter
        executeShooterAction(action);
    }

    /**
     * Check if navigation is at setpoint.
     */
    public boolean isNavigationComplete() {
        return navControllerX.atSetpoint() && navControllerY.atSetpoint();
    }

    /**
     * Check if turning is at setpoint.
     */
    public boolean isTurnComplete() {
        return turnController.atSetpoint();
    }

    // Getters for subsystems (for RobotContainer compatibility)
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public ShooterSubsystem getShooter() {
        return shooter;
    }

    public LocalizationSubsystem getLocalization() {
        return localization;
    }
}
