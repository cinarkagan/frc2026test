package frc.robot.controller.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants for autonomous routines.
 * Simplified version - uses DriveConstants for PID values.
 */
public final class AutoConstants {

    private AutoConstants() {}

    // =========== DRIVE CONSTRAINTS ===========
    /** Maximum velocity in meters per second */
    public static final double MAX_VELOCITY = 3.0;

    /** Maximum acceleration in meters per second squared */
    public static final double MAX_ACCELERATION = 2.0;

    /** Maximum angular velocity in radians per second */
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;

    // =========== TOLERANCES ===========
    /** Position tolerance in meters */
    public static final double POSITION_TOLERANCE = 0.05;

    /** Angle tolerance in degrees */
    public static final double ANGLE_TOLERANCE = 2.0;

    // =========== PATHPLANNER PID ===========
    /** Translation PID for PathPlanner - PLACEHOLDER, needs tuning */
    public static final double PP_TRANSLATION_P = 5.0;
    public static final double PP_TRANSLATION_I = 0.0;
    public static final double PP_TRANSLATION_D = 0.0;

    /** Rotation PID for PathPlanner - PLACEHOLDER, needs tuning */
    public static final double PP_ROTATION_P = 5.0;
    public static final double PP_ROTATION_I = 0.0;
    public static final double PP_ROTATION_D = 0.0;

    // =========== FIELD POSITIONS ===========
    public static final class FieldPositions {
        // Blue Alliance starting positions
        public static final Pose2d BLUE_START_LEFT = new Pose2d(0.75, 6.7, Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_START_CENTER = new Pose2d(1.4, 5.5, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_START_RIGHT = new Pose2d(0.75, 4.3, Rotation2d.fromDegrees(-60));

        // Blue wing notes
        public static final Pose2d BLUE_WING_NOTE_1 = new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_WING_NOTE_2 = new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_WING_NOTE_3 = new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(0));
    }
}
