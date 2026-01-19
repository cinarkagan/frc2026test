package frc.robot.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.constants.DriveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.robot.subsystems.LocalizationSubsystem;

/**
 * A command that will turn the robot to face the detected AprilTag.
 */
public class TurnToTag extends Command {
    private final PIDController controller;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final double maxAngularRate;

    public TurnToTag(CommandSwerveDrivetrain drivetrain, double maxAngularRate,
                     LocalizationSubsystem localization) {
        this.controller = new PIDController(
            DriveConstants.turnP,
            DriveConstants.turnI,
            DriveConstants.turnD
        );
        this.drivetrain = drivetrain;
        this.maxAngularRate = maxAngularRate;
        this.drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(
            DriveConstants.turnToleranceDeg,
            DriveConstants.turnToleranceDegPerSec
        );
        controller.setSetpoint(localization.getLimelightTagHeading());

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double output = controller.calculate(drivetrain.getGyroHeading());

        drivetrain.setControl(this.drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(output));
    }

    @Override
    public boolean isFinished() {
        // TODO: Enable when ready
        return false; // controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
