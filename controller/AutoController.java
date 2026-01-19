package frc.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controller.auto.AutoCommands;
import frc.robot.robot.state.RobotAction;
import frc.robot.robot.state.RobotState;
import frc.robot.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Autonomous controller that uses WPILib's native CommandScheduler.
 * 
 * This controller:
 * - Schedules commands through WPILib's CommandScheduler
 * - Uses SendableChooser for dashboard auto selection
 * - Integrates with PathPlanner when configured
 * 
 * Unlike the old implementation, this does NOT manually call command.execute().
 * Commands are scheduled once and run by WPILib automatically.
 */
public class AutoController implements Controller {

    private final CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Command> autoChooser;
    
    private Command currentCommand;
    private boolean hasScheduled = false;

    public AutoController(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.autoChooser = AutoCommands.createChooser(drivetrain);
    }

    // =========== COMMAND ACCESS ===========

    /**
     * Get the currently selected auto command from the dashboard.
     */
    public Command getSelectedCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : Commands.none();
    }

    /**
     * Get the SendableChooser for external access.
     */
    public SendableChooser<Command> getChooser() {
        return autoChooser;
    }

    /**
     * Get the drivetrain for creating custom commands.
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    // =========== CONTROLLER INTERFACE ===========

    @Override
    public String getName() {
        return "AutoController";
    }

    @Override
    public void onActivate() {
        // Get the selected command from dashboard
        currentCommand = getSelectedCommand();
        hasScheduled = false;

        if (currentCommand != null && currentCommand != Commands.none()) {
            // Schedule the command through WPILib's scheduler
            CommandScheduler.getInstance().schedule(currentCommand);
            hasScheduled = true;
            System.out.println("Scheduled auto: " + currentCommand.getName());
        } else {
            System.out.println("No auto routine selected!");
        }
    }

    @Override
    public void onDeactivate() {
        // Cancel any running auto command
        if (currentCommand != null && hasScheduled) {
            CommandScheduler.getInstance().cancel(currentCommand);
        }
        currentCommand = null;
        hasScheduled = false;
    }

    @Override
    public boolean isFinished() {
        if (currentCommand == null || !hasScheduled) {
            return true;
        }
        return currentCommand.isFinished();
    }

    @Override
    public RobotAction update(RobotState state) {
        // WPILib's CommandScheduler handles command execution automatically
        // We don't need to do anything here
        // The scheduler runs in Robot.robotPeriodic() via CommandScheduler.getInstance().run()
        return RobotAction.none();
    }

    // =========== CONVENIENCE METHODS ===========

    /**
     * Create a custom auto command using AutoCommands factory.
     */
    public Command createCustomAuto(java.util.function.Function<CommandSwerveDrivetrain, Command> factory) {
        return factory.apply(drivetrain);
    }

    /**
     * Add a custom auto option to the chooser.
     */
    public void addAutoOption(String name, Command command) {
        autoChooser.addOption(name, command);
    }
}
