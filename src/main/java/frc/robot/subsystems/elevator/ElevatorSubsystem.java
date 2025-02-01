package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_Elevator;
    private final SparkMaxConfig config = new SparkMaxConfig();

    public ElevatorSubsystem() {
        // Define the motor
        m_Elevator = new SparkMax(Constants.SubMotorIDs.kElevatorID, MotorType.kBrushless);

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
                // .positionConversionFactor(0) // need math for this. Rotations to IN/FT? RotationToMeters? -> (SpoolRadius * 2 * Math.PI)?
                // .velocityConversionFactor(0) // change RPM to Inch Per Sec / Feet Per Sec? RPM-MPS? -> (SpoolRadius * (2 * Math.PI)) / 60?
            ; // Leave this here to make config happy

        // Set the motor configuration
        m_Elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static class E_PIDF {
        public static final Constraints Constraints = new Constraints(0, 0);

        public static final double FeedForward = 0;

        public static final double Proportion = 0;

        public static final double Integral = 0;

        public static final double Derivative = 0;
    }





    public Command ManualStop() {
        return run(() -> 
            m_Elevator.set(0)
        );
    }
}