package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.PWM;


public class LEDControl {

    private AddressableLED ledStrip;
    private int ledLength = 30;  // Adjust to the length of your NeoPixel strip

    public LEDControl() {
        // Initialize the AddressableLED object on PWM port 0
        ledStrip = new AddressableLED(0);  // PWM Port (e.g., 0 for PWM 0)
        ledStrip.setLength(ledLength);     // Set the number of LEDs in the strip
        ledStrip.start();                  // Start the LED strip (necessary for it to work)
    }

    // Method to set color on all LEDs in the strip
    public void setColor(double redValue, double greenValue, double blueValue) {
        // Convert the color values (0-1 range) into integer values (0-255)
        int red = (int) (redValue * 255);
        int green = (int) (greenValue * 255);
        int blue = (int) (blueValue * 255);

        // Update the LED strip with the new color data
        setColor(red = 255, green = 0, blue = 0);
    }

    public static void main(String[] args) {
        LEDControl ledControl = new LEDControl();
        // Example: Set the LED color to dim white (50% red, 50% green, 50% blue)
        ledControl.setColor(0.5, 0.5, 0.5);
    }
}