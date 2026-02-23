package frc.robot.subsystems.localization;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/*
 Limelight(1st) Piplines:
 * 0 = Localization
 * 
 * 
*/

public class LocalizationSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    LimelightHelpers.PoseEstimate currentPoseEstimateFinal;
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
            LimelightHelpers.SetIMUMode("limelight-three",2);
            LimelightHelpers.SetIMUMode("limelight-four",2);
            LimelightHelpers.SetRobotOrientation("limelight-three", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.SetRobotOrientation("limelight-four", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate4_1;
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate4_2;
            
            if (useMT2) {
                limelightMeasurementEstimate4_1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-ffour");
                limelightMeasurementEstimate4_2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");
            } else {
                limelightMeasurementEstimate4_1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-ffour");
                limelightMeasurementEstimate4_2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");
            }
            /*if (limelightMeasurementEstimate!= null){SmartDashboard.putNumber("xll3a", limelightMeasurementEstimate.pose.getX());}
            if (limelightMeasurementEstimate4_2!= null){SmartDashboard.putNumber("xll4", limelightMeasurementEstimate4.pose.getX());}
            if (limelightMeasurementEstimate!= null&&limelightMeasurementEstimate.tagCount > 0){SmartDashboard.putNumber("ambiguity ll3a",limelightMeasurementEstimate.rawFiducials[0].ambiguity);}
            if (limelightMeasurementEstimate4_2!= null&&limelightMeasurementEstimate4.tagCount > 0){SmartDashboard.putNumber("ambiguity ll4",limelightMeasurementEstimate4.rawFiducials[0].ambiguity);}*/ 
            LimelightHelpers.PoseEstimate finalEstimate = null;
            boolean isLimelight4_1Seeing = (limelightMeasurementEstimate4_1!= null&&limelightMeasurementEstimate4_1.tagCount > 0);
            boolean isLimelight4_2Seeing = (limelightMeasurementEstimate4_2!= null&&limelightMeasurementEstimate4_2.tagCount > 0);
            if (isLimelight4_1Seeing) {
                if (limelightMeasurementEstimate4_1.rawFiducials[0].ambiguity > 0.7){
                    limelightMeasurementEstimate4_1 = null;
                } 
            }
            if (isLimelight4_2Seeing) {
                if (limelightMeasurementEstimate4_2.rawFiducials[0].ambiguity > 0.7){
                    limelightMeasurementEstimate4_2 = null;
                }
            }
            if (isLimelight4_1Seeing&&isLimelight4_2Seeing) {
                if (limelightMeasurementEstimate4_1.rawFiducials[0].ambiguity < 0.4 && limelightMeasurementEstimate4_2.rawFiducials[0].ambiguity < 0.4) {
                    double avgx = (limelightMeasurementEstimate4_1.pose.getX()+limelightMeasurementEstimate4_2.pose.getX())/2;
                    double avgy = (limelightMeasurementEstimate4_1.pose.getY()+limelightMeasurementEstimate4_2.pose.getY())/2;
                    double avgHeading = (limelightMeasurementEstimate4_1.pose.getRotation().getDegrees()+limelightMeasurementEstimate4_2.pose.getRotation().getDegrees())/2;
                    Pose2d avgPose2d = new Pose2d(new Translation2d(avgx,avgy),new Rotation2d(avgHeading));
                    finalEstimate = new LimelightHelpers.PoseEstimate(avgPose2d,limelightMeasurementEstimate4_1.timestampSeconds,limelightMeasurementEstimate4_1.latency,limelightMeasurementEstimate4_1.tagCount,limelightMeasurementEstimate4_1.tagSpan,limelightMeasurementEstimate4_1.avgTagDist,limelightMeasurementEstimate4_1.avgTagArea,limelightMeasurementEstimate4_1.rawFiducials,limelightMeasurementEstimate4_1.isMegaTag2);
                } else if (limelightMeasurementEstimate4_1.rawFiducials[0].ambiguity > limelightMeasurementEstimate4_2.rawFiducials[0].ambiguity) {
                    finalEstimate = limelightMeasurementEstimate4_2;
                } else {
                    finalEstimate = limelightMeasurementEstimate4_1;
                }
            } else if (isLimelight4_1Seeing) {
                finalEstimate = limelightMeasurementEstimate4_1;
            } else if (isLimelight4_2Seeing) {
                finalEstimate = limelightMeasurementEstimate4_2;
            } 

            if ((finalEstimate != null)&&(finalEstimate.tagCount > 0)) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        finalEstimate.pose,
                        finalEstimate.timestampSeconds);
                currentPoseEstimateFinal = finalEstimate;
                SmartDashboard.putNumber("finalx", finalEstimate.pose.getX());
            }
    }

    public boolean hasAnEstimate() {
        return (currentPoseEstimateFinal != null);
    }
    public double getLimelightTagHeading() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose.getRotation().getDegrees();}
        return 0;
    }

    public Pose2d getLimelightPose() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose;}
        return new Pose2d();
    }

    public LimelightHelpers.PoseEstimate getLimelightPoseEstimate() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal;}
        return new LimelightHelpers.PoseEstimate();
    }

    public Command useMT2() {
        return new InstantCommand(()->{useMT2 = true;});
    }

    
    public Command useMT() {
        return new InstantCommand(()->{useMT2 = false;});
    }
}
