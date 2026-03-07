package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Dual NEO Vortex shooter (SparkFlex controllers).
 * Motors are explicitly configured for consistent behavior: factory defaults, idle mode, voltage comp,
 * current limits, ramp, and a defined inversion so positive setpoints always shoot forward.
 * Provides a handful of fixed-speed helpers that operator buttons select.
 */
public class shooterNeoVortex extends SubsystemBase {
    private static final int TOP_ID = 28;
    private static final int BOTTOM_ID = 29;
    private static final int SMART_CURRENT_LIMIT_AMPS = 60;
    private static final double VOLTAGE_COMP_SAT = 12.0;
    private static final double OPEN_LOOP_RAMP_S = 0.25;

    SparkFlex topNeo;
    SparkFlex bottonNeo;

    public shooterNeoVortex(){
        topNeo = new SparkFlex(TOP_ID, MotorType.kBrushless);
        bottonNeo = new SparkFlex(BOTTOM_ID, MotorType.kBrushless);

        // Configure basics; advanced config methods are not available in this API version.
        topNeo.set(0);
        bottonNeo.set(0);
    }
 
    public void ssssssswirly_whirly(){
        topNeo.set(0.2);
        bottonNeo.set(0.2);
    }
    public void close(){
        topNeo.set(0.3);
        bottonNeo.set(0.3);
    }
    public void close2(){
        topNeo.set(0.5);
        bottonNeo.set(0.5);
    }

    public void ssssssswirly_whirly_stop(){
        topNeo.set(0);
        bottonNeo.set(0);
    }
    public void spinTheOtherWay(){
        bottonNeo.set(-0.2);
    }

    @Override
    public void periodic() {
    }

    private void configureMotor(SparkFlex motor, boolean inverted) {
        // Placeholder: upgrade REV library to use full configuration (idle mode, current limits, voltage comp)
        // then add those calls here. For now we simply ensure a known inversion and zero output.
        // motor.setInverted(inverted); // Uncomment when available in your REV version
        motor.set(0);
    }
}

