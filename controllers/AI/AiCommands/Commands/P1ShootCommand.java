package frc.robot.controllers.AI.AiCommands.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.LocationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSimulation;

public class P1ShootCommand extends SequentialCommandGroup {
    public P1ShootCommand(ShooterSimulation shooterSimulation) {
        super(
            AutoBuilder.pathfindToPose(LocationConstants.P1_POSE, new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720)), 0), //DriveToPose
            shooterSimulation.launchFuelCommand()
        );
    }
}
