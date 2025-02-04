package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

    public ElevatorSubsystem() {
        m_Elevator = new SparkMax(Constants.SubMotorIDs.kElevatorID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        m_Elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runElev(double setpoint) {
        return run(() -> {
            //See how far off you are
            var error = m_Elevator.getEncoder().getPosition() - setpoint;
            //calculate what motor power you need
            var kp_gain = 0.01; 
            var output = error*kp_gain;
            m_Elevator.set(output);
        });
    }
    
    public Command ManualStop() {
        return run(() -> 
            m_Elevator.set(0)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getEncoder().getPosition());
        
        if (m_bottomLimit.get()) {
            m_Elevator.getEncoder().setPosition(0);
        }
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