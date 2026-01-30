package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AI.AiCommands.AICommands;
import frc.robot.controllers.AI.AiCommands.CommandController;
import frc.robot.controllers.AI.AiCommands.Commands.P1ShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSimulation;

public class AiController implements Controller {
    CommandSwerveDrivetrain drivetrain;
    ShooterSimulation shooterSimulation;
    CommandController commandController;
    public AiController(CommandSwerveDrivetrain drivetrain, ShooterSimulation shooterSimulation) {
        this.drivetrain = drivetrain;
        this.shooterSimulation = shooterSimulation;
        this.commandController = new CommandController(this.shooterSimulation);
    }
    @Override
    public void getInitializeFunction() {
        commandController.execute(AICommands.P1_SHOOT);
    }
    @Override
    public void simulationPeriodic() {
    }
    @Override
    public void periodic() {
    }
    public Command getAutonomousCommand() {
        return new P1ShootCommand(shooterSimulation);
    }
    
}
