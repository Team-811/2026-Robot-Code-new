package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
   private SparkMax IndexerMotor;
        public Indexer(){
        IndexerMotor = new SparkMax(17, MotorType.kBrushless);
        }
    public void spin(){
        IndexerMotor.set(-0.4);
    }
    public void dontspin(){
        IndexerMotor.set(0);
    }
          @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
