package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_Elevator;
    private final SparkMaxConfig config = new SparkMaxConfig();
    
    private final ProfiledPIDController m_elePIDController = new ProfiledPIDController(E_PIDF.Proportion, E_PIDF.Integral, E_PIDF.Derivative, E_PIDF.Constraints);

    public ElevatorSubsystem() {
        // Define the motor
        m_Elevator = new SparkMax(Constants.SubMotorIDs.kElevatorID, MotorType.kBrushless);

        // Set the motor configs
        config
            // Deal with things with idle mode/follower first
            .idleMode(IdleMode.kBrake)
        
            .smartCurrentLimit(20)
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

        m_elePIDController.setTolerance(0.003);
        m_elePIDController.calculate(0);
    }

    public static class E_PIDF {
        public static final Constraints Constraints = new Constraints(0.3, 0.3);

        public static final double FeedForward = 0;

        public static final double Proportion = 0;

        public static final double Integral = 0;

        public static final double Derivative = 0;
    }

    public double getEncoderPos() {
        return m_Elevator.getAbsoluteEncoder().getPosition();
    }

    public Command runElevator(double setpoint) {
        m_elePIDController.setGoal(setpoint);

        return run(() -> {
            m_Elevator.set(m_elePIDController.calculate(getEncoderPos(), setpoint));
        })
        .until(() -> m_elePIDController.atSetpoint() && MathUtil.isNear(setpoint, getEncoderPos(), 0.003))
        .andThen(runOnce(() -> m_Elevator.set(0)));
    }

    public Command ManualRun(DoubleSupplier supplier) {
        double power = supplier.getAsDouble();
        return run(() ->
            m_Elevator.set(power)
        );
    }

    public Command ManualStop() {
        return run(() -> 
            m_Elevator.set(0)
        );
    }
}

