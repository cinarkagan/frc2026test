package frc.robot.robot.state;

import java.util.Optional;

/**
 * Represents an action request from a controller to the robot.
 * Controllers produce these; RobotCore consumes and executes them.
 */
public class RobotAction {

    public enum ActionType {
        NONE,
        DRIVE,
        NAVIGATE,
        TURN,
        SHOOTER,
        COMPOSITE
    }

    private final ActionType type;
    private final DriveAction driveAction;
    private final NavigateAction navigateAction;
    private final TurnAction turnAction;
    private final ShooterAction shooterAction;

    private RobotAction(ActionType type, DriveAction drive, NavigateAction nav,
                        TurnAction turn, ShooterAction shooter) {
        this.type = type;
        this.driveAction = drive;
        this.navigateAction = nav;
        this.turnAction = turn;
        this.shooterAction = shooter;
    }

    // =========== DRIVE ACTION ===========
    public static class DriveAction {
        public final double velocityX;
        public final double velocityY;
        public final double rotationRate;
        public final boolean fieldRelative;

        public DriveAction(double vx, double vy, double omega, boolean fieldRelative) {
            this.velocityX = vx;
            this.velocityY = vy;
            this.rotationRate = omega;
            this.fieldRelative = fieldRelative;
        }

        public static DriveAction stop() {
            return new DriveAction(0, 0, 0, true);
        }
    }

    // =========== NAVIGATE ACTION ===========
    public static class NavigateAction {
        public final double targetX;
        public final double targetY;
        public final Double targetHeading;

        public NavigateAction(double x, double y, Double heading) {
            this.targetX = x;
            this.targetY = y;
            this.targetHeading = heading;
        }

        public NavigateAction(double x, double y) {
            this(x, y, null);
        }
    }

    // =========== TURN ACTION ===========
    public static class TurnAction {
        public final double targetAngleDegrees;

        public TurnAction(double angle) {
            this.targetAngleDegrees = angle;
        }
    }

    // =========== SHOOTER ACTION ===========
    public static class ShooterAction {
        public enum ShooterCommand { NONE, ENABLE, DISABLE, ADJUST_RPM }

        public final ShooterCommand command;
        public final double rpmDelta;

        private ShooterAction(ShooterCommand cmd, double delta) {
            this.command = cmd;
            this.rpmDelta = delta;
        }

        public static ShooterAction enable() {
            return new ShooterAction(ShooterCommand.ENABLE, 0);
        }

        public static ShooterAction disable() {
            return new ShooterAction(ShooterCommand.DISABLE, 0);
        }

        public static ShooterAction adjustRPM(double delta) {
            return new ShooterAction(ShooterCommand.ADJUST_RPM, delta);
        }

        public static ShooterAction none() {
            return new ShooterAction(ShooterCommand.NONE, 0);
        }
    }

    // =========== FACTORY METHODS ===========
    public static RobotAction none() {
        return new RobotAction(ActionType.NONE, null, null, null, null);
    }

    public static RobotAction drive(double vx, double vy, double omega) {
        return new RobotAction(ActionType.DRIVE,
            new DriveAction(vx, vy, omega, true), null, null, null);
    }

    public static RobotAction driveRobotRelative(double vx, double vy, double omega) {
        return new RobotAction(ActionType.DRIVE,
            new DriveAction(vx, vy, omega, false), null, null, null);
    }

    public static RobotAction navigateTo(double x, double y) {
        return new RobotAction(ActionType.NAVIGATE,
            null, new NavigateAction(x, y), null, null);
    }

    public static RobotAction navigateTo(double x, double y, double heading) {
        return new RobotAction(ActionType.NAVIGATE,
            null, new NavigateAction(x, y, heading), null, null);
    }

    public static RobotAction turnTo(double angleDegrees) {
        return new RobotAction(ActionType.TURN,
            null, null, new TurnAction(angleDegrees), null);
    }

    public static RobotAction shooter(ShooterAction action) {
        return new RobotAction(ActionType.SHOOTER, null, null, null, action);
    }

    public static RobotAction composite(DriveAction drive, ShooterAction shooter) {
        return new RobotAction(ActionType.COMPOSITE, drive, null, null, shooter);
    }

    // =========== GETTERS ===========
    public ActionType getType() { return type; }
    public Optional<DriveAction> getDriveAction() { return Optional.ofNullable(driveAction); }
    public Optional<NavigateAction> getNavigateAction() { return Optional.ofNullable(navigateAction); }
    public Optional<TurnAction> getTurnAction() { return Optional.ofNullable(turnAction); }
    public Optional<ShooterAction> getShooterAction() { return Optional.ofNullable(shooterAction); }
}
