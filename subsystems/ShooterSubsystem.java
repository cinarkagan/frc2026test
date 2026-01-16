package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.fasterxml.jackson.databind.annotation.EnumNaming;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;


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
            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs_shooter.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs_shooter.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs_shooter.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs_shooter.Slot0.kI = 0; // No output for integrated error
        configs_shooter.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs_shooter.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        shooter1.getConfigurator().apply(configs_shooter);
        shooter2.getConfigurator().apply(configs_shooter);

            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs_feeder.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs_feeder.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs_feeder.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs_feeder.Slot0.kI = 0; // No output for integrated error
        configs_feeder.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs_feeder.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        feeder.getConfigurator().apply(configs_feeder);
    }

    @Override
    public void periodic() {
            //System.out.println(goalRPM);
            //System.out.println(shooter1.getVelocity().getValueAsDouble()*60);
            //System.out.println(shooter2.getVelocity().getValueAsDouble()*60);
            //System.out.println((shooter1.getVelocity().getValueAsDouble()*60)/(1.5));
            shooter1.setControl(m_velocityVoltage.withVelocity(-(goalRPM/60)));
            shooter2.setControl(m_velocityVoltage.withVelocity(goalRPM2/60));
            feeder.setControl(m_velocityVoltage.withVelocity(-(ShooterConstants.feederRPM/60)));
    }

    public Command enableShooter(){
        return new InstantCommand(()->{enabled = true;},this);
    }

    public Command disableShooter(){
        return new InstantCommand(()->{enabled = false;},this);
    }

    public Command increaseShooter(){
        return new InstantCommand(()->{goalRPM = goalRPM+100;},this);
    }
    public Command decreaseShooter(){
        return new InstantCommand(()->{goalRPM = goalRPM-100;},this);
    }
}
