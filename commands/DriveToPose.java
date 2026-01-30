package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TeleopConstants;
import frc.robot.controllers.Teleop;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
    PIDController controllerX;
    PIDController controllerY;
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric drive;
    double MaxAngularRate;
    double MaxDriveRate;
    public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        controllerX = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        controllerY = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        this.drivetrain = drivetrain;
        this.MaxDriveRate = TeleopConstants.MaxSpeed;
        this.MaxAngularRate = TeleopConstants.MaxAngularRate;
        this.drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxDriveRate*0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // Set the controller to be continuous (because it is an angle controller)
        //controllerX.enableContinuousInput(-1, 1);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        controllerX.setTolerance(DriveConstants.toleranceXCM/100);
        controllerX.setSetpoint(goalPose.getX());
        //controllerY.enableContinuousInput(-1, 1);
        controllerY.setTolerance(DriveConstants.toleranceYCM/100);
        controllerY.setSetpoint(goalPose.getY());
        addRequirements(drivetrain);
    }
    @Override
  public void execute() {
    Pose2d botPose = drivetrain.getSwerveDriveState().Pose;
    double outputX = controllerX.calculate(botPose.getX());
    double outputY = controllerX.calculate(botPose.getY());
    //outputY = MathUtil.clamp(outputY, -10, 10);
    //outputX = MathUtil.clamp(outputX, -10, 10);
    drivetrain.setControl(this.drive
      .withVelocityX(-outputX)
      .withVelocityY(-outputY)
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
