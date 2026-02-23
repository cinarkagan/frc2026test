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
    LimelightHelpers.PoseEstimate currentPoseEstimate;
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
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate;
            LimelightHelpers.PoseEstimate limelightMeasurementEstimate4;
            
            if (useMT2) {
                limelightMeasurementEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");
                limelightMeasurementEstimate4 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");
            } else {
                limelightMeasurementEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-three");
                limelightMeasurementEstimate4 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");
            }
            if (limelightMeasurementEstimate!= null){SmartDashboard.putNumber("xll3a", limelightMeasurementEstimate.pose.getX());}
            if (limelightMeasurementEstimate4!= null){SmartDashboard.putNumber("xll4", limelightMeasurementEstimate4.pose.getX());}
            if (limelightMeasurementEstimate!= null&&limelightMeasurementEstimate.tagCount > 0){SmartDashboard.putNumber("ambiguity ll3a",limelightMeasurementEstimate.rawFiducials[0].ambiguity);}
            if (limelightMeasurementEstimate4!= null&&limelightMeasurementEstimate4.tagCount > 0){SmartDashboard.putNumber("ambiguity ll4",limelightMeasurementEstimate4.rawFiducials[0].ambiguity);}   
            LimelightHelpers.PoseEstimate finalEstimate = null;
                        if (limelightMeasurementEstimate!= null&&limelightMeasurementEstimate.tagCount > 0) {

            if (limelightMeasurementEstimate.rawFiducials[0].ambiguity > 0.7){
                limelightMeasurementEstimate = null;
            } }
            if (limelightMeasurementEstimate4!= null&&limelightMeasurementEstimate4.tagCount > 0) {
            if (limelightMeasurementEstimate4.rawFiducials[0].ambiguity > 0.7){
                limelightMeasurementEstimate4 = null;
            }   }
            if (limelightMeasurementEstimate != null && limelightMeasurementEstimate4 != null&&limelightMeasurementEstimate4.tagCount > 0&&limelightMeasurementEstimate.tagCount > 0) {
                        
                if (limelightMeasurementEstimate.rawFiducials[0].ambiguity < 0.4 && limelightMeasurementEstimate.rawFiducials[0].ambiguity < 0.4) {
                    double avgx = (limelightMeasurementEstimate.pose.getX()+limelightMeasurementEstimate4.pose.getX())/2;
                    double avgy = (limelightMeasurementEstimate.pose.getY()+limelightMeasurementEstimate4.pose.getY())/2;
                    double avgHeading = (limelightMeasurementEstimate.pose.getRotation().getDegrees()+limelightMeasurementEstimate4.pose.getRotation().getDegrees())/2;
                    Pose2d avgPose2d = new Pose2d(new Translation2d(avgx,avgy),new Rotation2d(avgHeading));
                    finalEstimate = new LimelightHelpers.PoseEstimate(avgPose2d,limelightMeasurementEstimate.timestampSeconds,limelightMeasurementEstimate.latency,limelightMeasurementEstimate.tagCount,limelightMeasurementEstimate.tagSpan,limelightMeasurementEstimate.avgTagDist,limelightMeasurementEstimate.avgTagArea,limelightMeasurementEstimate.rawFiducials,limelightMeasurementEstimate.isMegaTag2);
                } else if (limelightMeasurementEstimate.rawFiducials[0].ambiguity > limelightMeasurementEstimate4.rawFiducials[0].ambiguity) {
                    finalEstimate = limelightMeasurementEstimate4;
                } else {
                    finalEstimate = limelightMeasurementEstimate;
                }
            } else if (limelightMeasurementEstimate != null&&limelightMeasurementEstimate.tagCount > 0) {
                finalEstimate = limelightMeasurementEstimate;
            } else if (limelightMeasurementEstimate4 != null&&limelightMeasurementEstimate4.tagCount > 0) {
                finalEstimate = limelightMeasurementEstimate4;
            } 

            if ((finalEstimate != null)&&(finalEstimate.tagCount > 0)) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        finalEstimate.pose,
                        finalEstimate.timestampSeconds);
                currentPoseEstimate = finalEstimate;
                SmartDashboard.putNumber("finalx", finalEstimate.pose.getX());
            }
    }

    public boolean hasAnEstimate() {
        return (currentPoseEstimate != null);
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
