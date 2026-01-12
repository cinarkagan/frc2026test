package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LocalizationSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
     @Override
    public void periodic() {
        double robotYaw = drivetrain.getGyroHeading();  
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        drivetrain.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds
    );
    }

    public double getFusedHeading() {
        double heading = LimelightHelpers.getBotPose_TargetSpace("")[2];
        return heading;
    }
}
