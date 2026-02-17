package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterKraken extends SubsystemBase{
    TalonFX amongus;
    public shooterKraken(int id){
        amongus = new TalonFX(id);
    }

    public void ssssssswirly_whirly(double sped){
        amongus.set(sped);
    }

    public void ssssssswirly_whirly_stop(){
        amongus.set(0);
    }

    @Override
   public void periodic() {
    
  }
}
