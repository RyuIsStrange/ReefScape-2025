package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_Elevator;
    private final SparkMaxConfig config = new SparkMaxConfig();

    public ElevatorSubsystem() {
        // Define the motor
        m_Elevator = new SparkMax(13, MotorType.kBrushless);

        // Define the Encoder

        // Set the motor configs
        config
            // Deal with things with idle mode/follower first
            .idleMode(IdleMode.kBrake)

            /* 
             * Move onto the encoder
             * Leave commented out till we decide if we should be using IPS or FPS
            */
            // .absoluteEncoder
                // .positionConversionFactor(0) // need math for this. Rotations to IN/FT? (SpoolRadius * 2 * Math.PI)?
                // .velocityConversionFactor(0) // change RPM to Inch Per Sec / Feet Per Sec? (SpoolRadius * 2 * Math.PI) / 60?
            ; // Leave this here to make config happy

        // Set the motor configuration
        m_Elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}