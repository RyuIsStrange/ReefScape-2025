package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_leftShooter;
    private final SparkMaxConfig config_Left = new SparkMaxConfig();

    private final SparkMax m_rightShooter;
    private final SparkMaxConfig config_Right = new SparkMaxConfig();

    public ShooterSubsystem() {
        // Define the motor
        m_leftShooter = new SparkMax(Constants.SubMotorIDs.kShooterMLeft, MotorType.kBrushless);

        // Set the motor configs
        config_Left
            .idleMode(IdleMode.kCoast)
            ; // Leave this here to make config happy 

        // Apply the motor configuration
        m_leftShooter.configure(config_Left, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ------------------------------------------------------------------------------------------------- //
        
        // Define the motor
        m_rightShooter = new SparkMax(Constants.SubMotorIDs.kShooterMRight, MotorType.kBrushless);
    
        // Set the motor configs
        config_Right
            .idleMode(IdleMode.kCoast)
            ; // Leave this here to make config happy

        // Apply the motor configuration
        m_rightShooter.configure(config_Right, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
