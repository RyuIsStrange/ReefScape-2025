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
    private final SparkMax m_Elevator;
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final DigitalInput m_bottomLimit = new DigitalInput(0);
    private final DigitalInput m_L1 = new DigitalInput(1);
    private final DigitalInput m_L2 = new DigitalInput(2);
    private final DigitalInput m_L3 = new DigitalInput(3);
    private final DigitalInput m_L4 = new DigitalInput(4);
    // private final DigitalInput m_upperLimit = new DigitalInput(5); // We dont have rn

    public ElevatorSubsystem() {
        m_Elevator = new SparkMax(Constants.SubMotorIDs.kElevatorID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        m_Elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public BooleanSupplier switchStates(DigitalInput limitSwitch) {
        return () -> limitSwitch.get() == false;
    }

    public Command runElevBtm() {
        return run(() -> {
            m_Elevator.set(-0.5);
        })
        .until(switchStates(m_bottomLimit)) // Go until bottom limit switch
        .andThen(ManualStop());
    }
    public Command runElevL1() {
        return runOnce(() -> 
            m_Elevator.set(.5)
        )
        .until(switchStates(m_L1)) // go until L1 switch
        .andThen(ManualStop());
    }
    public Command runElevL2() {
        return run(() -> {
            m_Elevator.set(.5);
        })
        .until(switchStates(m_L2)) // go until L2 switch
        .andThen(ManualStop());
    }
    public Command runElevL3() {
        return run(() -> {
            m_Elevator.set(.5);
        })
        .until(switchStates(m_L3)) // go until L3 switch
        .andThen(ManualStop());
    }
    public Command runElevL4() {
        return run(() -> {
            m_Elevator.set(.5);
        })
        .until(switchStates(m_L4)) // go until this one
        .andThen(ManualStop());
    }

    /**
     * Determines if the current level is above the target level
     */
    private boolean isAbove(DigitalInput current, DigitalInput target) {
        List<DigitalInput> levels = List.of(m_bottomLimit, m_L1, m_L2, m_L3, m_L4);
        return levels.indexOf(current) > levels.indexOf(target);
    }

    DigitalInput currentLevel = null;

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
            return ManualStop();
        }

        DigitalInput targetSwitch = levelMap.get(level);

        return run(() -> {
            // Find the current position by checking which switch is triggered first (false = triggered)
            if (!m_L4.get()) currentLevel = m_L4;
            else if (!m_L3.get()) currentLevel = m_L3;
            else if (!m_L2.get()) currentLevel = m_L2;
            else if (!m_L1.get()) currentLevel = m_L1;
            else if (!m_bottomLimit.get()) currentLevel = m_bottomLimit;
    
            if (currentLevel == null || currentLevel == targetSwitch) {
                m_Elevator.set(0); // Already at the desired level, stop motor
            } else if (isAbove(currentLevel, targetSwitch)) {
                m_Elevator.set(-1); // Move down
            } else {
                m_Elevator.set(1); // Move up
            }
        })
        .until(switchStates(targetSwitch))
        .andThen(ManualStop());
    }

    // Manual Controls

    public Command ManualRun(double speed) {
        return run(() -> m_Elevator.set(speed));
    }

    public Command ManualStop() {
        return run(() -> 
            m_Elevator.set(0)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getEncoder().getPosition());
        System.out.println(currentLevel);
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