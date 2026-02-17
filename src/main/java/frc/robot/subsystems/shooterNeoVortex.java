package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterNeoVortex extends SubsystemBase {
    SparkFlex what;
    public shooterNeoVortex(int id){
        what = new SparkFlex(id, null);
    }

    public void ssssssswirly_whirly(double speed){
        what.set(speed);
    }

    public void ssssssswirly_whirly_stop(){
        what.set(0);
    }

    @Override
   public void periodic() {
    
  }
}
