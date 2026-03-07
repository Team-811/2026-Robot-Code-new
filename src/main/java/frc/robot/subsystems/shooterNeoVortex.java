package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Dual NEO Vortex shooter (SparkFlex controllers).
 * Provides a handful of fixed-speed helpers that operator buttons select:
 * - ssssssswirly_whirly(): gentle intake-speed spin
 * - close(): mid-power shot
 * - close2(): full-power shot
 * - spinTheOtherWay(): reverse to clear jams
 * All helpers are open-loop percent output to keep behavior simple for driver practice.
 */
public class shooterNeoVortex extends SubsystemBase {
    SparkFlex topNeo;
    SparkFlex bottonNeo;
    public shooterNeoVortex(){
        topNeo = new SparkFlex(28, MotorType.kBrushless);
        bottonNeo = new SparkFlex(29, MotorType.kBrushless);

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
}
