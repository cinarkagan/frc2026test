package frc.robot.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooter1 = new TalonFX(ShooterConstants.shooter1_ID, ShooterConstants.canbus);
    private final TalonFX shooter2 = new TalonFX(ShooterConstants.shooter2_ID, ShooterConstants.canbus);
    private final TalonFX feeder = new TalonFX(ShooterConstants.feeder_ID, ShooterConstants.canbus);

    public boolean enabled = false;
    public double goalRPM = 2800;
    public double goalRPM2 = 3350;

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public TalonFXConfiguration configs_shooter = new TalonFXConfiguration();
    public TalonFXConfiguration configs_feeder = new TalonFXConfiguration();

    public ShooterSubsystem() {
        // Shooter motor configuration
        configs_shooter.Slot0.kS = 0.1;
        configs_shooter.Slot0.kV = 0.12;
        configs_shooter.Slot0.kP = 0.11;
        configs_shooter.Slot0.kI = 0;
        configs_shooter.Slot0.kD = 0;
        configs_shooter.Voltage
            .withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        shooter1.getConfigurator().apply(configs_shooter);
        shooter2.getConfigurator().apply(configs_shooter);

        // Feeder motor configuration
        configs_feeder.Slot0.kS = 0.1;
        configs_feeder.Slot0.kV = 0.12;
        configs_feeder.Slot0.kP = 0.11;
        configs_feeder.Slot0.kI = 0;
        configs_feeder.Slot0.kD = 0;
        configs_feeder.Voltage
            .withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        feeder.getConfigurator().apply(configs_feeder);
    }

    @Override
    public void periodic() {
        shooter1.setControl(m_velocityVoltage.withVelocity(-(goalRPM / 60)));
        shooter2.setControl(m_velocityVoltage.withVelocity(goalRPM2 / 60));
        feeder.setControl(m_velocityVoltage.withVelocity(-(ShooterConstants.feederRPM / 60)));
    }

    public Command enableShooter() {
        return new InstantCommand(() -> { enabled = true; }, this);
    }

    public Command disableShooter() {
        return new InstantCommand(() -> { enabled = false; }, this);
    }

    public Command increaseShooter() {
        return new InstantCommand(() -> { goalRPM = goalRPM + 100; }, this);
    }

    public Command decreaseShooter() {
        return new InstantCommand(() -> { goalRPM = goalRPM - 100; }, this);
    }
}
