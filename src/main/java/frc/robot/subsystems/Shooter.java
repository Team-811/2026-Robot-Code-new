// package frc.robot.subsystems;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// /**
//  * Simple two-motor shooter subsystem. Accepts percent outputs for top/bottom motors and
//  * publishes the last commanded values for debugging.
//  */
// public class Shooter extends SubsystemBase {
//   private final TalonFX topMotor = new TalonFX(Constants.OperatorConstants.shooterTopId);
//   private final TalonFX bottomMotor = new TalonFX(Constants.OperatorConstants.shooterBottomId);
//   private final DutyCycleOut topDuty = new DutyCycleOut(0);
//   private final DutyCycleOut bottomDuty = new DutyCycleOut(0);

//   private double lastTop = 0.0;
//   private double lastBottom = 0.0;

//   /**
//    * Set percent outputs for the shooter motors. Inputs are clamped to [-1, 1] to protect the
//    * controllers if callers overshoot while tuning. Values are stored for dashboard visibility.
//    */
//   public void setPercents(double topPercent, double bottomPercent) {
//     lastTop = clamp(topPercent);
//     lastBottom = clamp(bottomPercent);
//     topMotor.setControl(topDuty.withOutput(lastTop));
//     bottomMotor.setControl(bottomDuty.withOutput(lastBottom));
//   }

//   /** Stop both shooter motors. */
//   public void stop() {
//     setPercents(0.0, 0.0);
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Shooter/TopPercent", lastTop);
//     SmartDashboard.putNumber("Shooter/BottomPercent", lastBottom);
//   }

//   private double clamp(double value) {
//     return Math.max(-1.0, Math.min(1.0, value));
//   }
// }
