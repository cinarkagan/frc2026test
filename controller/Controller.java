package frc.robot.controller;

import frc.robot.robot.state.RobotState;
import frc.robot.robot.state.RobotAction;

/**
 * Interface for all robot controllers.
 * Controllers receive robot state and produce actions.
 * They have NO direct access to subsystems.
 */
public interface Controller {

    /**
     * Called when this controller becomes active.
     * Use for initialization/reset of internal state.
     */
    default void onActivate() {}

    /**
     * Called when this controller is deactivated.
     */
    default void onDeactivate() {}

    /**
     * Process the current robot state and produce an action.
     * Called every robot periodic loop (~20ms).
     *
     * @param state Current robot state snapshot
     * @return Action to execute on the robot
     */
    RobotAction update(RobotState state);

    /**
     * Get the name of this controller for logging/debugging.
     */
    String getName();

    /**
     * Check if the controller has completed its task.
     * Mainly useful for autonomous controllers.
     */
    default boolean isFinished() {
        return false;
    }
}
