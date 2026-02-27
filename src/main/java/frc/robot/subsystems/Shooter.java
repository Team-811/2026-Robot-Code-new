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
// // package frc.robot.subsystems;

// // import com.ctre.phoenix6.configs.Slot0Configs;
// // import com.ctre.phoenix6.controls.VelocityVoltage;
// // import com.ctre.phoenix6.hardware.TalonFX;

// // import edu.wpi.first.networktables.NetworkTableInstance;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // public class theActualShooter extends SubsystemBase{

// //     TalonFX intakee;
// //     private final double cH = 0;
// //     private final double H = 0; // change to measured values later
// //     private final VelocityVoltage velo = new VelocityVoltage(0);
// //     private final double targetV;
// //     public theActualShooter(){
// //         intakee = new TalonFX(54);

// //           Slot0Configs slot0 = new Slot0Configs();
// //         slot0.kP = 0.12;
// //         slot0.kI = 0.0;
// //         slot0.kD = 0.0;
// //         slot0.kV = 0.0; // feedforward

// //         intakee.getConfigurator().apply(slot0);
// //             double ty = NetworkTableInstance.getDefault()
// //         .getTable("limelight-lime")
// //         .getEntry("ty")
// //         .getDouble(0);

// //         double distance = (H - cH)/Math.tan(Math.toRadians(ty));
// //         targetV = getSpeed(distance);

// //         intakee.getConfigurator().apply(slot0);
// //     }
// //         public void go(){
// //         // intakee.set(0.25);
// //         intakee.setControl(velo.withVelocity(targetV));
  
// //     }
// //     public double getSpeed(double distance){
// //         if(distance < 0) // change to tested value
// //             return 0;
// //         else if(distance < 0)
// //             return 0; //if need add more if statments
// //         else 
// //             return 0;
// //     }
// //     // public void goTheOtherWay(){
// //     //     intakee.set(-1);
    
// //     // }
// //     public void stopIntake(){
// //         intakee.set(0);
  
// //     }
// //     @Override
// //     public void periodic(){

// //     }
// }
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor = new TalonFX(55, "*");
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final String LIMELIGHT_NAME = "limelight-shooter";

    private static final double LIMELIGHT_HEIGHT = 0.80;
    private static final double TARGET_HEIGHT = 2.10;
    private static final double LIMELIGHT_ANGLE = Units.degreesToRadians(25);

    private static final double VELOCITY_TOLERANCE_RPS = 1.5;

    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

    private final InterpolatingDoubleTreeMap distanceToRPM =
        new InterpolatingDoubleTreeMap();

    private double targetRPM = 0;

    public Shooter() {

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0.12;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0.12;

        shooterMotor.getConfigurator().apply(slot0);

        // distanceToRPM.put(1.0, -500.0);
        // distanceToRPM.put(2.5, -3500.0);
        distanceToRPM.put(2.0, -2000.0);
        // distanceToRPM.put(3.0, -2000.0);
    }

    public void runShooterWithLimelight() {
        double distance = getDistanceMeters();
        if (distance < 0) {
            stopShooter();
            return;
        }

        Double rpm = distanceToRPM.get(distance);
        if (rpm == null) return;

        targetRPM = rpm;

        shooterMotor.setControl(
            velocityRequest.withVelocity(targetRPM / 60.0)
        );
    }

    private double getDistanceMeters() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        if (!hasTarget) return -1;

        double ty = limelightTable.getEntry("ty").getDouble(0);

        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
            Math.tan(LIMELIGHT_ANGLE + Units.degreesToRadians(ty));
    }

    public boolean isAtSpeed() {
        double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
        double targetRPS = targetRPM / 60.0;

        return Math.abs(currentRPS - targetRPS) < VELOCITY_TOLERANCE_RPS;
    }

    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    public double getTargetRPM() {
        return targetRPM;
    }
    @Override
public void periodic() {
    double tv = NetworkTableInstance.getDefault()
        .getTable("limelight-shooter")
        .getEntry("tv")
        .getDouble(0);

    System.out.println("Limelight tv: " + tv);
}
}