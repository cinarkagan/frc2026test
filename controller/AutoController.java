package frc.robot.controller;

import frc.robot.robot.state.RobotState;
import frc.robot.robot.state.RobotAction;
import frc.robot.robot.state.RobotAction.DriveAction;
import frc.robot.robot.state.RobotAction.ShooterAction;

import java.util.ArrayList;
import java.util.List;

/**
 * State machine-based autonomous controller.
 * Executes a sequence of waypoints based on robot state feedback.
 */
public class AutoController implements Controller {

    // =========== STATE DEFINITIONS ===========
    public enum AutoState {
        IDLE,
        DRIVING_TO_POINT,
        TURNING,
        WAITING,
        SHOOTING,
        FINISHED
    }

    // =========== WAYPOINT CLASS ===========
    public static class Waypoint {
        public final double x;
        public final double y;
        public final Double heading;
        public final WaypointAction action;

        public enum WaypointAction { NONE, SHOOT, WAIT }

        public Waypoint(double x, double y) {
            this(x, y, null, WaypointAction.NONE);
        }

        public Waypoint(double x, double y, Double heading) {
            this(x, y, heading, WaypointAction.NONE);
        }

        public Waypoint(double x, double y, Double heading, WaypointAction action) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.action = action;
        }
    }

    // =========== STATE ===========
    private AutoState currentState = AutoState.IDLE;
    private final List<Waypoint> waypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;

    // Tolerances
    private final double positionTolerance = 0.15;
    private final double angleTolerance = 3.0;

    // Timing
    private double stateStartTime = 0;
    private double waitDuration = 2.0;
    private double shootDuration = 2.0;

    // Current target
    private Waypoint currentTarget = null;
    private Double turnTarget = null;

    public AutoController() {}

    // =========== WAYPOINT MANAGEMENT ===========
    public AutoController addWaypoint(double x, double y) {
        waypoints.add(new Waypoint(x, y));
        return this;
    }

    public AutoController addWaypoint(double x, double y, double heading) {
        waypoints.add(new Waypoint(x, y, heading));
        return this;
    }

    public AutoController addWaypointWithShoot(double x, double y, double heading) {
        waypoints.add(new Waypoint(x, y, heading, Waypoint.WaypointAction.SHOOT));
        return this;
    }

    public AutoController addWait(double seconds) {
        waypoints.add(new Waypoint(Double.NaN, Double.NaN, null, Waypoint.WaypointAction.WAIT));
        this.waitDuration = seconds;
        return this;
    }

    public void clearWaypoints() {
        waypoints.clear();
        currentWaypointIndex = 0;
        currentState = AutoState.IDLE;
        currentTarget = null;
        turnTarget = null;
    }

    // =========== CONTROLLER INTERFACE ===========
    @Override
    public String getName() {
        return "AutoController";
    }

    @Override
    public void onActivate() {
        currentWaypointIndex = 0;
        stateStartTime = 0;
        if (!waypoints.isEmpty()) {
            transitionToWaypoint(0);
        } else {
            currentState = AutoState.FINISHED;
        }
    }

    @Override
    public void onDeactivate() {
        currentState = AutoState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return currentState == AutoState.FINISHED;
    }

    @Override
    public RobotAction update(RobotState state) {
        // Initialize state start time if needed
        if (stateStartTime == 0) {
            stateStartTime = state.getTimestamp();
        }

        switch (currentState) {
            case IDLE:
                return RobotAction.none();

            case DRIVING_TO_POINT:
                return handleDrivingToPoint(state);

            case TURNING:
                return handleTurning(state);

            case WAITING:
                return handleWaiting(state);

            case SHOOTING:
                return handleShooting(state);

            case FINISHED:
                return RobotAction.drive(0, 0, 0);

            default:
                return RobotAction.none();
        }
    }

    // =========== STATE HANDLERS ===========
    private RobotAction handleDrivingToPoint(RobotState state) {
        if (currentTarget == null) {
            advanceToNextWaypoint(state);
            return RobotAction.none();
        }

        double distanceToTarget = state.distanceTo(currentTarget.x, currentTarget.y);

        // Check if we've arrived
        if (distanceToTarget < positionTolerance) {
            // Check if we need to turn to a specific heading
            if (currentTarget.heading != null) {
                turnTarget = currentTarget.heading;
                currentState = AutoState.TURNING;
                stateStartTime = state.getTimestamp();
                return RobotAction.turnTo(turnTarget);
            }

            // Check for action at waypoint
            if (currentTarget.action == Waypoint.WaypointAction.SHOOT) {
                currentState = AutoState.SHOOTING;
                stateStartTime = state.getTimestamp();
                return RobotAction.shooter(ShooterAction.enable());
            }

            // Advance to next waypoint
            advanceToNextWaypoint(state);
            return RobotAction.none();
        }

        // Navigate to target
        if (currentTarget.heading != null) {
            return RobotAction.navigateTo(currentTarget.x, currentTarget.y, currentTarget.heading);
        } else {
            return RobotAction.navigateTo(currentTarget.x, currentTarget.y);
        }
    }

    private RobotAction handleTurning(RobotState state) {
        if (turnTarget == null) {
            advanceToNextWaypoint(state);
            return RobotAction.none();
        }

        double headingError = Math.abs(normalizeAngle(turnTarget - state.getHeadingDegrees()));

        if (headingError < angleTolerance) {
            // Done turning, check for action
            if (currentTarget != null && currentTarget.action == Waypoint.WaypointAction.SHOOT) {
                currentState = AutoState.SHOOTING;
                stateStartTime = state.getTimestamp();
                return RobotAction.shooter(ShooterAction.enable());
            }

            advanceToNextWaypoint(state);
            return RobotAction.none();
        }

        return RobotAction.turnTo(turnTarget);
    }

    private RobotAction handleWaiting(RobotState state) {
        double elapsed = state.getTimestamp() - stateStartTime;

        if (elapsed >= waitDuration) {
            advanceToNextWaypoint(state);
            return RobotAction.none();
        }

        return RobotAction.drive(0, 0, 0);
    }

    private RobotAction handleShooting(RobotState state) {
        double elapsed = state.getTimestamp() - stateStartTime;

        if (elapsed >= shootDuration) {
            advanceToNextWaypoint(state);
            return RobotAction.shooter(ShooterAction.disable());
        }

        // Keep shooter enabled, hold position
        return RobotAction.composite(
            new DriveAction(0, 0, 0, true),
            ShooterAction.enable()
        );
    }

    // =========== HELPERS ===========
    private void transitionToWaypoint(int index) {
        if (index >= waypoints.size()) {
            currentState = AutoState.FINISHED;
            currentTarget = null;
            return;
        }

        currentTarget = waypoints.get(index);

        if (currentTarget.action == Waypoint.WaypointAction.WAIT) {
            currentState = AutoState.WAITING;
        } else {
            currentState = AutoState.DRIVING_TO_POINT;
        }
    }

    private void advanceToNextWaypoint(RobotState state) {
        currentWaypointIndex++;
        stateStartTime = state.getTimestamp();
        transitionToWaypoint(currentWaypointIndex);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // =========== DEBUG/TELEMETRY ===========
    public AutoState getCurrentState() {
        return currentState;
    }

    public int getCurrentWaypointIndex() {
        return currentWaypointIndex;
    }

    public int getTotalWaypoints() {
        return waypoints.size();
    }
}
