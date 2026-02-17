// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.PositionVoltage;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OperatorConstants;


// public class climber extends SubsystemBase{
//     TalonSRX johnSRX;
//       PIDController elPID ;
//   final PositionVoltage request;
//   public Encoder encoder;
//   private double target;
//     public climber(){
//           johnSRX = new TalonSRX(15);
//               johnSRX.setPosition(0);
//        var slot0Configs = new Slot0Configs();
//     slot0Configs.kV = 0;
//     slot0Configs.kA = 0;
//    slot0Configs.kP = 0.3;
//    slot0Configs.kI = 0;
//    slot0Configs.kD = 0.05;
//    slot0Configs.kG =0;
//    johnSRX.getConfigurator().apply(slot0Configs);
//     request = new PositionVoltage(0).withSlot(0);

//     }
//        public void spinthatjohn(){
//         johnSRX.set(TalonSRXControlMode.PercentOutput, 0.5);
//     }
    
//   public void turnPoint(){
//     // System.out.println(elMotor.getRotorPosition().getValue());
//     if(Math.abs(target-johnSRX.getRotorPosition().getValueAsDouble())>OperatorConstants.kElDeadBand){
//       johnSRX.setControl(request.withPosition(target));
//     }
//     else{
//         johnSRX.set(TalonSRXControlMode.PercentOutput, 0);
//     }
    

//   }
//   public boolean setPoint(double goal){
//     target= goal;
//     System.out.println("hello");
    
//     return Math.abs(target-johnSRX.getRotorPosition().getValueAsDouble())<OperatorConstants.kElDeadBand;
//   }

//   public void stopElevator(){
//     johnSRX.set(0);
//   }

//     @Override
    // public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
