package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.MachineStates;
import frc.robot.utils.AllStates.ShooterStates;

public class MachineSubsystem {
    LocalizationSubsystem localizationSubsystem;
    ShooterSubsystem shooterSubsystem;

    public AllStates.MachineStates  currentState = MachineStates.IDLE;
    public AllStates.MachineStates requestedState = MachineStates.IDLE;
    CommandSwerveDrivetrain drivetrain;
    public MachineSubsystem(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.shooterSubsystem = new ShooterSubsystem();
        this.localizationSubsystem = new LocalizationSubsystem(drivetrain);
    }

    public void requestState(MachineStates state){requestedState = state;}
    public void customShootRequest(double rpm){
        requestState(MachineStates.CUSTOM_SHOOT);
        shooterSubsystem.customRequest(rpm);
    }
    public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
    public LocalizationSubsystem getLocalizationSubsystem() { return localizationSubsystem; }
}
