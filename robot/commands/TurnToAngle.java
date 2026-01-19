package frc.robot.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.constants.DriveConstants;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that will turn the robot to the specified angle.
 */
public class TurnToAngle extends Command {
    private final PIDController controller;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final double maxAngularRate;

    public TurnToAngle(double targetAngleDegrees, CommandSwerveDrivetrain drivetrain,
                       SwerveRequest.FieldCentric drive, double maxAngularRate) {
        this.controller = new PIDController(
            DriveConstants.turnP,
            DriveConstants.turnI,
            DriveConstants.turnD
        );
        this.drivetrain = drivetrain;
        this.maxAngularRate = maxAngularRate;
        this.drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(maxAngularRate * 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        controller.setTolerance(
            Math.toRadians(DriveConstants.turnToleranceDeg),
            Math.toRadians(DriveConstants.turnToleranceDegPerSec)
        );
        controller.setSetpoint(Math.toRadians(targetAngleDegrees));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double output = controller.calculate(Math.toRadians(drivetrain.getGyroHeading()));
        output = MathUtil.clamp(output, -maxAngularRate, maxAngularRate);

        drivetrain.setControl(this.drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(output));

        System.out.println(Math.toDegrees(controller.getError()));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
