package frc.robot.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.constants.DriveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.robot.subsystems.LocalizationSubsystem;

/**
 * A command that drives the robot toward the detected AprilTag.
 */
public class DriveToTag extends Command {
    private final PIDController controllerX;
    private final PIDController controllerY;
    private final LocalizationSubsystem localizationSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final double maxAngularRate;
    private final double maxDriveRate;

    public DriveToTag(CommandSwerveDrivetrain drivetrain, double maxDriveRate,
                      double maxAngularRate, LocalizationSubsystem localization) {
        this.controllerX = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );
        this.controllerY = new PIDController(
            DriveConstants.driveP,
            DriveConstants.driveI,
            DriveConstants.driveD
        );

        this.drivetrain = drivetrain;
        this.maxDriveRate = maxDriveRate;
        this.maxAngularRate = maxAngularRate;
        this.localizationSubsystem = localization;
        this.drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxDriveRate * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        controllerX.enableContinuousInput(-1, 1);
        controllerX.setTolerance(
            DriveConstants.turnToleranceDeg,
            DriveConstants.turnToleranceDegPerSec
        );
        controllerX.setSetpoint(DriveConstants.goalX);

        controllerY.enableContinuousInput(-1, 1);
        controllerY.setTolerance(
            DriveConstants.turnToleranceDeg,
            DriveConstants.turnToleranceDegPerSec
        );
        controllerY.setSetpoint(DriveConstants.goalY);
    }

    @Override
    public void execute() {
        double outputX = controllerX.calculate(localizationSubsystem.getLimelightPose().getX());
        double outputY = controllerY.calculate(localizationSubsystem.getLimelightPose().getY());

        outputX = MathUtil.clamp(outputX, -0.3, 0.3);
        outputY = MathUtil.clamp(outputY, -0.3, 0.3);

        drivetrain.setControl(this.drive
            .withVelocityX(outputX)
            .withVelocityY(outputY)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return controllerX.atSetpoint() && controllerY.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
