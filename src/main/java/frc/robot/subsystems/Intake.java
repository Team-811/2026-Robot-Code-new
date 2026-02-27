package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   private SparkMax IntakeMotor;
    public Intake(){
        IntakeMotor = new SparkMax(16, MotorType.kBrushless);
    }
    public void spin(){
        IntakeMotor.set(1);
    }
    public void dontspin(){
        IntakeMotor.set(0);
    }
          @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
