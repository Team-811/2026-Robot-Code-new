package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Dual NEO Vortex shooter (SparkFlex controllers).
 * Open-loop percent output with clamped outputs. SparkFlex config APIs are unavailable in this
 * REVLib version, so advanced settings (idle mode, current limit, voltage comp) are omitted.
 * Both motors are always commanded together.
 */
public class shooterNeoVortex extends SubsystemBase {
    private static final int TOP_ID = 28;
    private static final int BOTTOM_ID = 29;

    // Open-loop speed presets (percent output). Adjust as you tune shots.
    private static final double SPEED_INTAKE = 0.2;
    private static final double SPEED_CLOSE  = 0.3;
    private static final double SPEED_FULL   = 0.5;
    private static final double SPEED_CLEAR  = -0.2; // reverse both wheels to clear

    SparkFlex topNeo;
    SparkFlex bottonNeo;

    public shooterNeoVortex(){
        topNeo = new SparkFlex(TOP_ID, MotorType.kBrushless);
        bottonNeo = new SparkFlex(BOTTOM_ID, MotorType.kBrushless);

        topNeo.set(0);
        bottonNeo.set(0);
    }
 
    public void ssssssswirly_whirly(){
        setBoth(SPEED_INTAKE);
    }
    public void close(){
        setBoth(SPEED_CLOSE);
    }
    public void close2(){
        setBoth(SPEED_FULL);
    }

    public void ssssssswirly_whirly_stop(){
        setBoth(0);
    }
    public void spinTheOtherWay(){
        setBoth(SPEED_CLEAR);
    }

    @Override
    public void periodic() {
    }

    /** Clamp and apply the same percent output to both shooter motors. */
    private void setBoth(double percent) {
        double clamped = Math.max(-1.0, Math.min(1.0, percent));
        topNeo.set(clamped);
        bottonNeo.set(clamped);
    }
}

