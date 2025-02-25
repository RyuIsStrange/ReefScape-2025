package frc.robot.subsystems;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_Elevator; // Define the sparkmax for elevator
    private final SparkMaxConfig config = new SparkMaxConfig(); // Init the sparkMax config
    private final DigitalInput m_bottomLimit = new DigitalInput(0); // Init bottom limitswitch
    private final DigitalInput m_L1 = new DigitalInput(1); // Init L1 limit
    private final DigitalInput m_L2 = new DigitalInput(2); // Init L2 limit
    private final DigitalInput m_L3 = new DigitalInput(3); // Init L3 limit
    private final DigitalInput m_L4 = new DigitalInput(4); // Init L4 Limit
    // private final DigitalInput m_upperLimit = new DigitalInput(5); // We are just going to use L4 as upper limit

    public ElevatorSubsystem() {
        m_Elevator = new SparkMax(Constants.SubMotorIDs.kElevatorID, MotorType.kBrushless); // Actually set the values for Elevator
        config.idleMode(IdleMode.kBrake); // Set Elevator motor to brake mode
        m_Elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // Apply the config
    }

    // Make the BooleanSupplier check into a function so I dont have to put it into every command that needs it
    public BooleanSupplier switchStates(DigitalInput limitSwitch) {
        return () -> limitSwitch.get() == false;
    }

    // Manual Controls
    
    // Run the motor with the double speed
    public Command ManualRun(double speed) {
        return run(() -> m_Elevator.set(speed));
    }
    // Stop the motor
    public Command Stop() {
        return run(() -> 
            m_Elevator.set(0)
        );
    }

    // New Elevator command (Non-Auto)

    /**
     * Determines if the current level is above the target level
     */
    private boolean isAbove(DigitalInput current, DigitalInput target) {
        List<DigitalInput> levels = List.of(m_bottomLimit, m_L1, m_L2, m_L3, m_L4);
        return levels.indexOf(current) > levels.indexOf(target);
    }

    // Define this outside of the command so that there is a "currentLevel" value of null so it doesn't freak out
    DigitalInput currentLevel = null;

    // Define the command
    /*
     * I need to test if you can just use this off the rip or if you actually have to press bottom before running anything
     * Like we have been doing to make sure it "knows" were it is.
     */
    public Command NewEle(String level) {
        // Map level names to corresponding limit switches
        Map<String, DigitalInput> levelMap = Map.of(
            "Bottom", m_bottomLimit,
            "L1", m_L1,
            "L2", m_L2,
            "L3", m_L3,
            "L4", m_L4
        );

        // Ensure the requested level exists
        if (!levelMap.containsKey(level)) {
            return Stop();
        }

        // Get the target switch from the map
        DigitalInput targetSwitch = levelMap.get(level);

        // run the fun things
        return run(() -> {
            // Find the current position by checking which switch is triggered first (false = triggered)
            if (!m_L4.get()) currentLevel = m_L4;
            else if (!m_L3.get()) currentLevel = m_L3;
            else if (!m_L2.get()) currentLevel = m_L2;
            else if (!m_L1.get()) currentLevel = m_L1;
            else if (!m_bottomLimit.get()) currentLevel = m_bottomLimit;
    
            if (currentLevel == null || currentLevel == targetSwitch) {
                m_Elevator.set(0); // Already at the desired level, stop motor or nothing has been defined as currentLevel
            } else if (isAbove(currentLevel, targetSwitch)) {
                m_Elevator.set(-0.9); // Move down
            } else {
                m_Elevator.set(1); // Move up
            }
        })
        .until(switchStates(targetSwitch)) // Until we reach target
        .andThen(Stop()); // Then stop
    }

    // "Old" Elevator commands (Auto)
    // I just didn't feel like having to define some other things for auto

    public Command runElevBtm() {
        return run(() -> {
            m_Elevator.set(-0.9);
        })
        .until(switchStates(m_bottomLimit)) // Go until bottom limit switch
        .andThen(Stop());
    }
    public Command runElevL1() {
        return runOnce(() -> 
            m_Elevator.set(.9)
        )
        .until(switchStates(m_L1)) // go until L1 switch
        .andThen(Stop());
    }
    public Command runElevL2() {
        return run(() -> {
            m_Elevator.set(.9);
        })
        .until(switchStates(m_L2)) // go until L2 switch
        .andThen(Stop());
    }
    public Command runElevL3() {
        return run(() -> {
            m_Elevator.set(.9);
        })
        .until(switchStates(m_L3)) // go until L3 switch
        .andThen(Stop());
    }
    public Command runElevL4() {
        return run(() -> {
            m_Elevator.set(.9);
        })
        .until(switchStates(m_L4)) // go until this one
        .andThen(Stop());
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getEncoder().getPosition()); // Have this for testing reasons
        SmartDashboard.putData("Current Level", currentLevel); // Push this to dashboard so I can debug faster @ comp
        System.out.println(currentLevel); // For testing, comment out later in Comp
    }
}



 /* 
             * Move onto the encoder
             * Leave commented out till we decide if we should be using IPS or FPS
            */
            // .absoluteEncoder
                // .positionConversionFactor(0) // need math for this. Rotations to IN/FT? RotationToMeters? -> (SpoolRadius * 2 * Math.PI)?
                // .velocityConversionFactor(0) // change RPM to Inch Per Sec / Feet Per Sec? RPM-MPS? -> (SpoolRadius * (2 * Math.PI)) / 60?
                ; // Leave this here to make config happy