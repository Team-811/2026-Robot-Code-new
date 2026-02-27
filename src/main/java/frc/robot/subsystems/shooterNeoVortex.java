package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterNeoVortex extends SubsystemBase {
    static double ssssssswirly_whirlyspeed = 0.45;
    SparkFlex what;
    SparkFlex what2;
    public shooterNeoVortex(){
        what = new SparkFlex(28, MotorType.kBrushless);
        what2 = new SparkFlex(29, MotorType.kBrushless);
    }
 
    public void ssssssswirly_whirly(){
        what.set(0.2);
        what2.set(0.2);
    }
    public void close(){
        what.set(0.3);
        what2.set(0.3);
    }
    public void close2(){
        what.set(0.1);
        what2.set(0.1);
    }

    public void ssssssswirly_whirly_stop(){
        what.set(0);
        what2.set(0);
    }
    public void spinTheOtherWay(){
        what2.set(-0.2);
    }

    public static void raiseSpeed(){
        ssssssswirly_whirlyspeed+=0.05;
        System.out.println("The neo vortex speed has been raised to "+ssssssswirly_whirlyspeed);
    }
    public static void lowerSpeed(){
        ssssssswirly_whirlyspeed-=0.05;
        System.out.println("The neo vortex speed has been lowered to "+ssssssswirly_whirlyspeed);
    }
    @Override
   public void periodic() {
    
  }
}
