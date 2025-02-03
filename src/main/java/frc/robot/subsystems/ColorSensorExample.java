package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensorExample {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 color;

    public ColorSensorExample() {
        color = new ColorSensorV3(i2cPort);
    }
}
