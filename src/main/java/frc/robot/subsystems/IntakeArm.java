// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class IntakeArm extends SubsystemBase{
//     private SparkMax intakePiv;
//     private SparkMaxConfig motorconfig;
//     private SparkClosedLoopController control;
//     private RelativeEncoder encoder;
//     private double setPoint;
//     public IntakeArm(){

//         intakePiv = new SparkMax(0, MotorType.kBrushless);
//         motorconfig = new SparkMaxConfig();
//         control = intakePiv.getClosedLoopController();
//         encoder = intakePiv.getEncoder();
//         motorconfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
//         motorconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         .p(0)
//         .i(0)
//         .d(0);

//         intakePiv.configure(motorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//     }
  
//     public void pivot(){
//             if(Math.abs(setPoint-intakePiv.getEncoder().getPosition())>1){
//       control.setSetpoint(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
//     }
//     else{
//       intakePiv.set(0);
// //     }
//     }
//     }
//     public void pivotGo(double setPoint){
//         this.setPoint = setPoint;
//     }
//     public void stop(){
  
//         intakePiv.set(0);
//     }
//       @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
    
// }
