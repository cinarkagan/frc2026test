package frc.robot.controllers.AI.AiCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AI.AiCommands.Commands.P1ShootCommand;
import frc.robot.subsystems.shooter.ShooterSimulation;


public class CommandController {
    ShooterSimulation shooterSimulation;
    public CommandController(ShooterSimulation shooterSimulation) {
        this.shooterSimulation = shooterSimulation;
    }
    public void execute(AICommands aiCommand) {
        switch(aiCommand) {
            case P1_SHOOT:
                new P1ShootCommand(shooterSimulation).schedule();
                break;
            case INTAKE:
                break;
            case CLIMB:
                break;
            default:
                break;
        }
    }
}
