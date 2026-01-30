package frc.robot.subsystems.shooter;

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
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.ShooterStates;


public class ShooterSubsystem extends SubsystemBase implements Shooter {
    
    private final TalonFX shooter1 = new TalonFX(ShooterConstants.shooter1_ID, ShooterConstants.canbus);
    private final TalonFX shooter2 = new TalonFX(ShooterConstants.shooter2_ID, ShooterConstants.canbus);
    private final TalonFX feeder = new TalonFX(ShooterConstants.feeder_ID, ShooterConstants.canbus);

    public double goalRPM = ShooterConstants.IDLE_RPM;
    public double goalRPM2 = -ShooterConstants.IDLE_RPM;
    public double customRPM = 3500;
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public TalonFXConfiguration configs_shooter = new TalonFXConfiguration();
    public TalonFXConfiguration configs_feeder = new TalonFXConfiguration();
    public AllStates.ShooterStates currentState = ShooterStates.IDLE;
    public AllStates.ShooterStates requestedState = ShooterStates.IDLE;

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
            stateMachine();
            shooter1.setControl(m_velocityVoltage.withVelocity(-(goalRPM*1.5/60)));
            shooter2.setControl(m_velocityVoltage.withVelocity(goalRPM2*2/60));
            feeder.setControl(m_velocityVoltage.withVelocity(-(ShooterConstants.feederRPM/60)));
    }

    public void requestState(ShooterStates state){requestedState = state;}
    public void customRequest(double rpm){
        requestState(ShooterStates.CUSTOM);
        customRPM = rpm;
    }
    public boolean isAtRPM(){
        if ((shooter1.getVelocity().getValueAsDouble()*60>(goalRPM-ShooterConstants.rpmTol))&&((shooter1.getVelocity().getValueAsDouble())*60<(goalRPM+ShooterConstants.rpmTol))){
            if ((shooter2.getVelocity().getValueAsDouble()*60>(goalRPM-ShooterConstants.rpmTol))&&((shooter2.getVelocity().getValueAsDouble())*60<(goalRPM+ShooterConstants.rpmTol))){
                return true;
            }
        }
        return false;
    }

    public void stateMachine() {
        if (currentState == requestedState){
                switch(currentState) {
                    case IDLE:
                        goalRPM = ShooterConstants.IDLE_RPM;
                        goalRPM2 = -ShooterConstants.IDLE_RPM;
                        break;
                    case KILL:
                        goalRPM = 0;
                        goalRPM2 = 0;
                        break;
                    case P1_SHOOT:
                        goalRPM = ShooterConstants.P1_RPM;
                        goalRPM2 = -ShooterConstants.P1_RPM;
                        break;
                    case P2_SHOOT:
                        goalRPM = ShooterConstants.P2_RPM;
                        goalRPM2 = -ShooterConstants.P2_RPM;
                        break;
                    case LOW_POWER:
                        goalRPM = ShooterConstants.LOW_POWER_RPM;
                        goalRPM2 = -ShooterConstants.LOW_POWER_RPM;
                        break;
                    case CUSTOM:
                        goalRPM = customRPM;
                        goalRPM2 = -customRPM;
                        break;

                }
        } else {
            currentState = requestedState;
            stateMachine();
        }
    }
    

    public Command increaseShooter(){
        return new InstantCommand(()->{customRequest(customRPM+100);},this);
    }
    public Command decreaseShooter(){
        return new InstantCommand(()->{customRequest(customRPM-100);},this);
    }

    @Override
    public double getRPM() {
        return (shooter1.getVelocity().getValueAsDouble()*60);
    }
    @Override
    public double getRPMHood() {
        return (shooter2.getVelocity().getValueAsDouble()*60);
    }
    @Override
    public double getRPMFlywheel() {
        return (shooter1.getVelocity().getValueAsDouble()*60/1.5);
    }
}
