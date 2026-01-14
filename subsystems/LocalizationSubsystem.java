package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/*
 Limelight(1st) Piplines:
 * 0 = Localization
 * 
 * 
*/

public class LocalizationSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    LimelightHelpers.PoseEstimate currentPoseEstimate;
    Pose2d currentPose2d;


    public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        drivetrain.gyroReset();
    }

    @Override
    public void periodic() {
        //LimelightHelpers.setPipelineIndex("", 0); // sonra sil
        //if (LimelightHelpers.getCurrentPipelineIndex("") == 0) {
            double robotYaw = drivetrain.getGyroHeading();
            //System.out.println(robotYaw);
            LimelightHelpers.SetIMUMode("",2);
            LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            Pose2d limelightPose2d = LimelightHelpers.getBotPose2d("");
            //LimelightHelpers.
            /*if (limelightMeasurement != null) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds);
                currentPoseEstimate = limelightMeasurement;
            }*/
            currentPoseEstimate = limelightMeasurementEstimate;
            currentPose2d = limelightPose2d;

            //System.out.println(LimelightHelpers.getTY(""));
        //}
        System.out.println(getLimelightPose().getX());
        System.out.println(getLimelightPose().getY());
    }

    public boolean hasAnEstimate() {
        return (currentPoseEstimate != null);
    }
    public boolean hasABotPose() {
        return (currentPose2d != null);
    }

    public double getLimelightTagHeading() {
        if (hasAnEstimate()) {return currentPoseEstimate.pose.getRotation().getDegrees();}
        return 0;
    }

    public Pose2d getLimelightPose() {
        if (hasAnEstimate()) {return currentPoseEstimate.pose;}
        return new Pose2d();
    }

    public LimelightHelpers.PoseEstimate getLimelightPoseEstimate() {
        if (hasAnEstimate()) {return currentPoseEstimate;}
        return new LimelightHelpers.PoseEstimate();
    }
}
