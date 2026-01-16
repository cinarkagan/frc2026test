package frc.robot.commands;

import org.opencv.features2d.FastFeatureDetector;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends Command {
  PIDController controller;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.FieldCentric drive;
  double MaxAngularRate;
  public TurnToAngle(double targetAngleDegrees, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, double MaxAngularRate) {
    /*super(
        new PIDController(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD),
        drivetrain::getHeading,
        targetAngleDegrees,
        output -> drivetrain.applyRequest(() ->
                drive.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(output)
        ),
        drivetrain
    );*/
    controller = new PIDController(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD);
    this.drivetrain = drivetrain;
    this.MaxAngularRate = MaxAngularRate;
    this.drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Set the controller to be continuous (because it is an angle controller)
    controller.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    controller.setTolerance(DriveConstants.turnToleranceDeg, DriveConstants.turnToleranceDegPerSec);
    controller.setSetpoint(targetAngleDegrees);

    
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
    // End when the controller is at the reference.
    if (controller.atSetpoint())
    {
      System.out.println(controller.getError());
    }
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle()); //idle when finished
  }

}