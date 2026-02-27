package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeForNow extends SubsystemBase{
    SparkMax intake;
    public intakeForNow(){
        intake =  new SparkMax(20, MotorType.kBrushless);
    }
    public void spin(){
        intake.set(1);
    }
    public void spinTheOtherWay(){
        intake.set(-1.0);
    }
    public void dontSpin(){
        intake.set(0);
    }
     @Override
    public void periodic() {}
}
