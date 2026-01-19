package frc.robot.controller.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;

// PathPlanner imports - uncomment when PathPlanner is added to vendordeps
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;

/**
 * Factory class for creating autonomous commands.
 * Provides simple methods to create WPILib Commands that are scheduled natively.
 * 
 * Usage:
 * <pre>
 * Command auto = AutoCommands.sequence(
 *     AutoCommands.driveToPose(drivetrain, new Pose2d(2, 0, new Rotation2d())),
 *     AutoCommands.wait(0.5),
 *     AutoCommands.driveToPose(drivetrain, new Pose2d(2, 2, Rotation2d.fromDegrees(90)))
 * );
 * </pre>
 */
public final class AutoCommands {

    private AutoCommands() {}

    // =========== DRIVE COMMANDS ===========

    /**
     * Create a command to drive to the specified pose.
     */
    public static Command driveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        return new DriveToPoseCommand(drivetrain, targetPose)
            .withName("DriveTo(" + formatPose(targetPose) + ")");
    }

    /**
     * Create a command to drive to the specified pose with custom speed limits.
     */
    public static Command driveToPose(
            CommandSwerveDrivetrain drivetrain, 
            Pose2d targetPose,
            double maxSpeed,
            double maxAngularSpeed) {
        return new DriveToPoseCommand(drivetrain, targetPose, maxSpeed, maxAngularSpeed)
            .withName("DriveTo(" + formatPose(targetPose) + ")");
    }

    /**
     * Create a command to drive to the specified coordinates.
     */
    public static Command driveTo(
            CommandSwerveDrivetrain drivetrain,
            double x, double y, double headingDegrees) {
        Pose2d target = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
        return driveToPose(drivetrain, target);
    }

    // =========== PATHPLANNER COMMANDS ===========

    /**
     * Follow a PathPlanner path by name.
     * The path file must exist in deploy/pathplanner/paths/
     * 
     * TODO: Uncomment when PathPlanner is added to vendordeps
     */
    public static Command followPath(String pathName) {
        // try {
        //     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        //     return AutoBuilder.followPath(path)
        //         .withName("FollowPath(" + pathName + ")");
        // } catch (Exception e) {
        //     return Commands.print("Failed to load path: " + pathName);
        // }

        // Placeholder until PathPlanner is configured
        return Commands.print("PathPlanner not configured: " + pathName)
            .andThen(Commands.waitSeconds(1.0))
            .withName("FollowPath(" + pathName + ")");
    }

    /**
     * Follow a PathPlanner path and reset odometry to the path's starting pose.
     */
    public static Command followPathWithReset(CommandSwerveDrivetrain drivetrain, String pathName) {
        // try {
        //     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        //     return Commands.sequence(
        //         Commands.runOnce(() -> drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))),
        //         AutoBuilder.followPath(path)
        //     ).withName("FollowPath(" + pathName + ")");
        // } catch (Exception e) {
        //     return Commands.print("Failed to load path: " + pathName);
        // }

        return followPath(pathName);
    }

    /**
     * Build a full auto from PathPlanner GUI.
     * The auto must be created in PathPlanner.
     */
    public static Command buildAuto(String autoName) {
        // return AutoBuilder.buildAuto(autoName);

        return Commands.print("PathPlanner auto not configured: " + autoName)
            .withName("Auto(" + autoName + ")");
    }

    // =========== UTILITY COMMANDS ===========

    /**
     * Wait for the specified duration.
     */
    public static Command wait(double seconds) {
        return Commands.waitSeconds(seconds)
            .withName("Wait(" + seconds + "s)");
    }

    /**
     * Run commands in sequence.
     */
    public static Command sequence(Command... commands) {
        return Commands.sequence(commands)
            .withName("Sequence[" + commands.length + "]");
    }

    /**
     * Run commands in parallel (all at once, finish when all complete).
     */
    public static Command parallel(Command... commands) {
        return Commands.parallel(commands)
            .withName("Parallel[" + commands.length + "]");
    }

    /**
     * Race commands (all at once, finish when first completes).
     */
    public static Command race(Command... commands) {
        return Commands.race(commands)
            .withName("Race[" + commands.length + "]");
    }

    /**
     * Run a command with a deadline (another command's completion).
     */
    public static Command deadline(Command deadline, Command... others) {
        return Commands.deadline(deadline, others);
    }

    /**
     * Reset the robot's odometry to the specified pose.
     */
    public static Command resetPose(CommandSwerveDrivetrain drivetrain, Pose2d pose) {
        return Commands.runOnce(() -> drivetrain.resetPose(pose))
            .withName("ResetPose(" + formatPose(pose) + ")");
    }

    // =========== EXAMPLE AUTOS ===========

    /**
     * Simple taxi auto - just drive forward.
     */
    public static Command taxiAuto(CommandSwerveDrivetrain drivetrain, Pose2d startPose) {
        return sequence(
            resetPose(drivetrain, startPose),
            driveTo(drivetrain, startPose.getX() + 2.0, startPose.getY(), 
                    startPose.getRotation().getDegrees())
        ).withName("Taxi Auto");
    }

    /**
     * Two pose auto - drive to two positions.
     */
    public static Command twoPoseAuto(
            CommandSwerveDrivetrain drivetrain,
            Pose2d startPose,
            Pose2d pose1,
            Pose2d pose2) {
        return sequence(
            resetPose(drivetrain, startPose),
            driveToPose(drivetrain, pose1),
            wait(0.25),
            driveToPose(drivetrain, pose2)
        ).withName("Two Pose Auto");
    }

    // =========== SENDABLE CHOOSER ===========

    /**
     * Create a SendableChooser with example autonomous routines.
     */
    public static SendableChooser<Command> createChooser(CommandSwerveDrivetrain drivetrain) {
        SendableChooser<Command> chooser = new SendableChooser<>();

        // Default - do nothing
        chooser.setDefaultOption("None", Commands.none());

        // Simple autos
        chooser.addOption("Taxi - Center", 
            taxiAuto(drivetrain, AutoConstants.FieldPositions.BLUE_START_CENTER));
        
        chooser.addOption("Taxi - Left",
            taxiAuto(drivetrain, AutoConstants.FieldPositions.BLUE_START_LEFT));

        // Add PathPlanner autos when configured
        // chooser.addOption("PP - 2 Note", buildAuto("2NoteAuto"));

        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }

    // =========== HELPERS ===========

    private static String formatPose(Pose2d pose) {
        return String.format("%.1f,%.1f,%.0f°", 
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
}
