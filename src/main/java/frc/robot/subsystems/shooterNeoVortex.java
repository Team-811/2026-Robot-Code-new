package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
