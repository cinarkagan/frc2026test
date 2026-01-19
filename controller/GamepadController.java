package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot.state.RobotState;
import frc.robot.robot.state.RobotAction;
import frc.robot.robot.state.RobotAction.DriveAction;
import frc.robot.robot.state.RobotAction.ShooterAction;

/**
 * Controller that translates Xbox gamepad inputs to robot actions.
 */
public class GamepadController implements Controller {

    private final CommandXboxController joystick;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final double deadband;

    // Track button states for edge detection
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    public GamepadController(CommandXboxController joystick,
                             double maxSpeed, double maxAngularRate) {
        this.joystick = joystick;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.deadband = 0.1;
    }

    @Override
    public String getName() {
        return "GamepadController";
    }

    @Override
    public RobotAction update(RobotState state) {
        // Read joystick values
        double leftY = -joystick.getLeftY();
        double leftX = -joystick.getLeftX();
        double rightX = -joystick.getRightX();

        // Apply deadband
        leftY = applyDeadband(leftY);
        leftX = applyDeadband(leftX);
        rightX = applyDeadband(rightX);

        // Scale to velocities
        double velocityX = leftY * maxSpeed;
        double velocityY = leftX * maxSpeed;
        double rotationRate = rightX * maxAngularRate;

        // Build drive action
        DriveAction driveAction = new DriveAction(velocityX, velocityY, rotationRate, true);

        // Check for shooter adjustments (edge detection)
        ShooterAction shooterAction = ShooterAction.none();

        boolean currentLeftBumper = joystick.getHID().getLeftBumperButton();
        boolean currentRightBumper = joystick.getHID().getRightBumperButton();

        if (currentLeftBumper && !lastLeftBumper) {
            shooterAction = ShooterAction.adjustRPM(-100);
        } else if (currentRightBumper && !lastRightBumper) {
            shooterAction = ShooterAction.adjustRPM(100);
        }

        lastLeftBumper = currentLeftBumper;
        lastRightBumper = currentRightBumper;

        // A button triggers turn-to-angle (66 degrees)
        if (joystick.getHID().getAButton()) {
            return RobotAction.turnTo(66);
        }

        // Return composite action if shooter changed, otherwise just drive
        if (shooterAction.command != ShooterAction.ShooterCommand.NONE) {
            return RobotAction.composite(driveAction, shooterAction);
        }

        return RobotAction.drive(velocityX, velocityY, rotationRate);
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.signum(value) * (Math.abs(value) - deadband) / (1.0 - deadband);
    }
}
