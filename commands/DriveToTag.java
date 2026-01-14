package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LocalizationSubsystem;

public class DriveToTag extends Command {
    PIDController controllerX;
    PIDController controllerY;
    LocalizationSubsystem localizationSubsystem;
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric drive;
    double MaxAngularRate;
    double MaxDriveRate;
    public DriveToTag(CommandSwerveDrivetrain drivetrain, double MaxDriveRate, double MaxAngularRate,LocalizationSubsystem localization) {
        controllerX = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        controllerY = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        this.drivetrain = drivetrain;
        this.MaxDriveRate = MaxDriveRate;
        this.MaxAngularRate = MaxAngularRate;
        this.localizationSubsystem = localization;
        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxDriveRate*0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // Set the controller to be continuous (because it is an angle controller)
        controllerX.enableContinuousInput(-1, 1);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        controllerX.setTolerance(DriveConstants.turnToleranceDeg, DriveConstants.turnToleranceDegPerSec);
        controllerX.setSetpoint(DriveConstants.goalX);
        controllerY.enableContinuousInput(-1, 1);
        controllerY.setTolerance(DriveConstants.turnToleranceDeg, DriveConstants.turnToleranceDegPerSec);
        controllerY.setSetpoint(DriveConstants.goalY);
    }
    @Override
  public void execute() {
    double outputX = controllerX.calculate(this.localizationSubsystem.getLimelightPose().getX());
    double outputY = controllerX.calculate(this.localizationSubsystem.getLimelightPose().getY());
    outputY = MathUtil.clamp(outputY, -0.3, 0.3);
    outputX = MathUtil.clamp(outputX, -0.3, 0.3);
    drivetrain.setControl(this.drive
      .withVelocityX(outputX)
      .withVelocityY(outputY)
      .withRotationalRate(0));
  }
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (controllerX.atSetpoint()&&controllerY.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle()); //idle when finished
  }
}
