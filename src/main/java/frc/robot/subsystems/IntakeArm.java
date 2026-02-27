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
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // public class IntakeArm extends SubsystemBase{
// //     private SparkMax intakePiv;
// //     private SparkMaxConfig motorconfig;
// //     private SparkClosedLoopController control;
// //     private RelativeEncoder encoder;
// //     private double setPoint;
// //     public IntakeArm(){

// //         intakePiv = new SparkMax(20, MotorType.kBrushless);
// //         motorconfig = new SparkMaxConfig();
// //         control = intakePiv.getClosedLoopController();
// //         encoder = intakePiv.getEncoder();
// //         motorconfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
// //         motorconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
// //         .p(0.7)
// //         .i(0)
// //         .d(0.2);

// //         intakePiv.configure(motorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

// //     }
  
// //     public void pivot(){
// //             if(Math.abs(setPoint-intakePiv.getEncoder().getPosition())>1){
// //       control.setSetpoint(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
// //     }
// //     else{
// //       intakePiv.set(0);
// // //     }
// //     }
// //     }
// //     public void pivotGo(double setPoint){
// //         this.setPoint = setPoint;
// //     }
// //     public void stop(){
  
// //         intakePiv.set(0);
// //     }
// //       @Override
// //   public void periodic() {
// //     // This method will be called once per scheduler run
// //     control.setSetpoint(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

// //   }
    
// // }




// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class IntakeArm extends SubsystemBase {

//     private final SparkMax motor;
//     private final SparkClosedLoopController controller;
//     private final RelativeEncoder encoder;

//     // Desired setpoint in rotations
//     private double setpoint = 0;

//     // PID constants (tune these for your arm)
//     private static final double kP = 1.5;
//     private static final double kI = 0.0;
//     private static final double kD = 0.2;

//     // Feedforward to overcome gravity (adjust between 0.03–0.1)
//     private static final double kG = 0.05;

//     public IntakeArm() {
//         // Create motor
//         motor = new SparkMax(20, MotorType.kBrushless);

//         // Get encoder and controller
//         encoder = motor.getEncoder();
//         controller = motor.getClosedLoopController();

//         // Configure motor
//         SparkMaxConfig config = new SparkMaxConfig();

//         // Encoder scaling
//         config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

//         // PID configuration
//         config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//             .p(kP).i(kI).d(kD);

//         // Set idle mode to brake
//         config.idleMode(IdleMode.kBrake);

//         // Apply configuration
//         motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//         // Zero encoder at startup
//         encoder.setPosition(0);
//     }

//     /**
//      * Set the desired arm position (rotations)
//      */
//     public void setPosition(double position) {
//         setpoint = position;
//     }

//     /**
//      * Get current arm position
//      */
//     public double getPosition() {
//         return encoder.getPosition();
//     }

//     /**
//      * Optional: stop motor output
//      */
//     public void stop() {
//         motor.set(0);
//     }

//     @Override
//     public void periodic() {
//         // Run PID with feedforward every scheduler cycle
//         controller.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, kG);

//         // Debug info
//         SmartDashboard.putNumber("IntakeArm Position", getPosition());
//         SmartDashboard.putNumber("IntakeArm Setpoint", setpoint);
//     }
// }