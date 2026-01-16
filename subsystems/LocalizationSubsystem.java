package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    boolean useMT2;


    public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.useMT2 = true;
        drivetrain.gyroReset();
    }

    @Override
    public void periodic() {
        //LimelightHelpers.setPipelineIndex("", 0); // sonra sil
        //if (LimelightHelpers.getCurrentPipelineIndex("") == 0) {
            double robotYaw = drivetrain.getGyroHeading();
            //sSystem.out.println(robotYaw);
            LimelightHelpers.SetIMUMode("",2);
            LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate;
            if (useMT2) {
                limelightMeasurementEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
            } else {
                limelightMeasurementEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            }
            Pose2d limelightPose2d = LimelightHelpers.getBotPose2d("");

            if ((limelightMeasurementEstimate != null)&&(limelightMeasurementEstimate.tagCount > 0)) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        limelightMeasurementEstimate.pose,
                        limelightMeasurementEstimate.timestampSeconds);
                currentPoseEstimate = limelightMeasurementEstimate;
            }            currentPose2d = limelightPose2d;

            //System.out.println(LimelightHelpers.getTY(""));
        //}
        //System.out.print("X");
        //System.out.println(getLimelightPose().getX());
        //System.out.print("Y");
        //System.out.println(getLimelightPose().getY());
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

    public Command useMT2() {
        return new InstantCommand(()->{useMT2 = true;});
    }

    
    public Command useMT() {
        return new InstantCommand(()->{useMT2 = false;});
    }
}
