package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeForNow extends SubsystemBase{
    SparkMax intake;
    SparkMaxConfig motorConfig;
    SparkClosedLoopController closedLoopController;
    public intakeForNow(){
        intake =  new SparkMax(20, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
  motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0);

//   intake.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
