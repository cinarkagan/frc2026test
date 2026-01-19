package frc.robot.robot.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Immutable snapshot of the robot's current state.
 * Passed to controllers each iteration for decision-making.
 * Controllers should NEVER modify this - they produce RobotAction instead.
 */
public class RobotState {
    // Position & Kinematics
    private final double positionX;
    private final double positionY;
    private final double headingDegrees;
    private final double velocityX;
    private final double velocityY;
    private final double angularVelocity;

    // Shooter State
    private final double shooterTargetRPM1;
    private final double shooterTargetRPM2;
    private final boolean shooterEnabled;

    // Vision State
    private final boolean hasVisionEstimate;
    private final int visibleTagCount;
    private final Pose2d visionPose;

    // Timing
    private final double timestamp;

    private RobotState(Builder builder) {
        this.positionX = builder.positionX;
        this.positionY = builder.positionY;
        this.headingDegrees = builder.headingDegrees;
        this.velocityX = builder.velocityX;
        this.velocityY = builder.velocityY;
        this.angularVelocity = builder.angularVelocity;
        this.shooterTargetRPM1 = builder.shooterTargetRPM1;
        this.shooterTargetRPM2 = builder.shooterTargetRPM2;
        this.shooterEnabled = builder.shooterEnabled;
        this.hasVisionEstimate = builder.hasVisionEstimate;
        this.visibleTagCount = builder.visibleTagCount;
        this.visionPose = builder.visionPose;
        this.timestamp = builder.timestamp;
    }

    // Getters
    public double getPositionX() { return positionX; }
    public double getPositionY() { return positionY; }
    public double getHeadingDegrees() { return headingDegrees; }
    public double getVelocityX() { return velocityX; }
    public double getVelocityY() { return velocityY; }
    public double getAngularVelocity() { return angularVelocity; }
    public double getShooterTargetRPM1() { return shooterTargetRPM1; }
    public double getShooterTargetRPM2() { return shooterTargetRPM2; }
    public boolean isShooterEnabled() { return shooterEnabled; }
    public boolean hasVisionEstimate() { return hasVisionEstimate; }
    public int getVisibleTagCount() { return visibleTagCount; }
    public Pose2d getVisionPose() { return visionPose; }
    public double getTimestamp() { return timestamp; }

    // Convenience methods
    public Pose2d getPose() {
        return new Pose2d(positionX, positionY, Rotation2d.fromDegrees(headingDegrees));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(velocityX, velocityY, angularVelocity);
    }

    public double distanceTo(double x, double y) {
        return Math.hypot(x - positionX, y - positionY);
    }

    public double angleTo(double x, double y) {
        return Math.toDegrees(Math.atan2(y - positionY, x - positionX));
    }

    // Builder class
    public static class Builder {
        private double positionX = 0;
        private double positionY = 0;
        private double headingDegrees = 0;
        private double velocityX = 0;
        private double velocityY = 0;
        private double angularVelocity = 0;
        private double shooterTargetRPM1 = 0;
        private double shooterTargetRPM2 = 0;
        private boolean shooterEnabled = false;
        private boolean hasVisionEstimate = false;
        private int visibleTagCount = 0;
        private Pose2d visionPose = new Pose2d();
        private double timestamp = 0;

        public Builder positionX(double val) { positionX = val; return this; }
        public Builder positionY(double val) { positionY = val; return this; }
        public Builder headingDegrees(double val) { headingDegrees = val; return this; }
        public Builder velocityX(double val) { velocityX = val; return this; }
        public Builder velocityY(double val) { velocityY = val; return this; }
        public Builder angularVelocity(double val) { angularVelocity = val; return this; }
        public Builder shooterTargetRPM1(double val) { shooterTargetRPM1 = val; return this; }
        public Builder shooterTargetRPM2(double val) { shooterTargetRPM2 = val; return this; }
        public Builder shooterEnabled(boolean val) { shooterEnabled = val; return this; }
        public Builder hasVisionEstimate(boolean val) { hasVisionEstimate = val; return this; }
        public Builder visibleTagCount(int val) { visibleTagCount = val; return this; }
        public Builder visionPose(Pose2d val) { visionPose = val; return this; }
        public Builder timestamp(double val) { timestamp = val; return this; }

        public RobotState build() {
            return new RobotState(this);
        }
    }
}
