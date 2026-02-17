package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class john extends SubsystemBase {

    private final TalonSRX john;
    
    // Motor & encoder constants
    private static final int TALON_ID = 27;
    private static final double ENCODER_CPR = 44.4; // encoder counts per motor rev
    private static final double GEAR_RATIO = 22.2;   // motor:output
    private static final double TARGET_DEGREES = 90; // degrees we want to rotate

    // computed target in encoder counts
    private final double targetCounts;

    public john() {
        john = new TalonSRX(TALON_ID);

        // Configure Talon to use a quadrature encoder
        john.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

        // Zero the encoder at startup
        john.setSelectedSensorPosition(0);

        // Simple PIDF constants for position control (adjust as needed)
        john.config_kP(0, 1.0, 10);
        john.config_kI(0, 0.0, 10);
        john.config_kD(0, 0.0, 10);
        john.config_kF(0, 0.0, 10);

        // compute counts for 90 degrees
        targetCounts = (ENCODER_CPR * GEAR_RATIO) * (TARGET_DEGREES / 360.0);
    }

    /** Command the motor to go to 90 degrees */
    public void moveTo90Degrees() {
        john.set(ControlMode.Position, targetCounts);
    }

    /** Stop the motor */
    public void stopMotor() {
        john.set(ControlMode.PercentOutput, 0);
    }

    /** Returns current position in degrees */
    public double getCurrentDegrees() {
        double counts = john.getSelectedSensorPosition();
        return (counts / (ENCODER_CPR * GEAR_RATIO)) * 360.0;
    }

    @Override
    public void periodic() {
        // Optional: print position to SmartDashboard
        // SmartDashboard.putNumber("Arm Degrees", getCurrentDegrees());
    }
}

