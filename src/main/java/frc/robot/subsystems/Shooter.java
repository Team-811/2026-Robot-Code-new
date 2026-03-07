package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Falcon-based shooter that maps Limelight distance estimates to RPM setpoints.
 * Designed for novice readability: every config is explicit so behavior is stable across reboots.
 * - Uses CTRE velocity closed-loop (with feedforward) instead of percent output so ball speed holds as battery sags.
 * - Distance->RPM map is clamped and stops the motor if no valid lookup exists to avoid mystery shots.
 * - Telemetry publishes target/actual RPM and ready flag to SmartDashboard so drivers know when to feed.
 */
public class Shooter extends SubsystemBase {

    // Hardware wiring/config (change here if IDs/bus change)
    private static final String CAN_BUS = "CANivore"; // Bus name: "CANivore" if plugged into CANivore, "rio" if on roboRIO CAN
    private static final int SHOOTER_ID = 55;         // TalonFX CAN ID for the shooter motor

    // Motor protection/consistency (tune as needed)
    private static final double SUPPLY_LIMIT_AMPS = 50.0; // Typical 40-60A to prevent brownouts; raise if shots lag, lower if browning out
    private static final double STATOR_LIMIT_AMPS = 80.0; // Torque limit; 60-100A is common for shooters
    private static final double CLOSED_LOOP_RAMP_S = 0.1; // Seconds to ramp velocity setpoint; increase for smoother spin-up
    private static final double VOLTAGE_COMP_SAT = 12.0;  // Voltage compensation target; keep at battery nominal (10-12V)

    private static final CANBus SHOOTER_CANBUS = new CANBus(CAN_BUS);
    private final TalonFX shooterMotor = new TalonFX(SHOOTER_ID, SHOOTER_CANBUS);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final String LIMELIGHT_NAME = "limelight-shooter"; // NetworkTables name for the aiming camera

 //   private static final double defaultSpeed = 1800; // RPM to use if you want a safe fallback when no distance is available

    // How close the motor velocity must be to the target to consider "ready" (rad/s). Lower = stricter, higher = more forgiving.
    private static final double VELOCITY_TOLERANCE_RPS = 1.5;

    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

    private final InterpolatingDoubleTreeMap distanceToRPM =
        new InterpolatingDoubleTreeMap();

    private double targetRPM = 0;

    public Shooter() {

        // Velocity closed-loop gains. Run SysId to replace these with measured kS/kV/kA and tuned kP.
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0.12;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0.12;

        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(slot0)
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)) // keep RPM table positive; flip if wiring requires
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_LIMIT_AMPS)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_LIMIT_AMPS)
                .withStatorCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs().withPeakForwardVoltage(VOLTAGE_COMP_SAT).withPeakReverseVoltage(-VOLTAGE_COMP_SAT))
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CLOSED_LOOP_RAMP_S));

        shooterMotor.getConfigurator().apply(cfg);

        // Distance (m) -> shooter RPM. Add more points as you calibrate; values should be positive (inversion set above).
        distanceToRPM.put(1.0, -750.0);
        distanceToRPM.put(2.0, -1960.0);
        distanceToRPM.put(3.0, -2650.0);
    }

    public void runShooterWithLimelight() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        if (!hasTarget) {
            // No valid target: stop to avoid firing at unknown speed
            targetRPM = 0;
            stopShooter();
            return;
        }

        double distance = getDistanceMeters();
        Double rpm = distanceToRPM.get(distance);
        if (rpm == null) {
            stopShooter();
            return;
        }

        targetRPM = rpm;

        // Command the TalonFX in velocity mode using RPM->RPS conversion; keeps closed-loop on the motor controller.
        shooterMotor.setControl(
            velocityRequest.withVelocity(targetRPM / 60.0)
        );
    }

    private double getDistanceMeters() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        if (!hasTarget) {
            return -1.0;
        }

        // Use camera-space pose for direct range to the observed AprilTag; this updates every frame.
        var pose = LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_NAME);
        if (pose == null) {
            return -1.0;
        }
        // Distance from camera to the observed AprilTag (meters, 3D norm).
        return pose.getTranslation().getNorm();
    }

    public boolean isAtSpeed() {
        double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
        double targetRPS = targetRPM / 60.0;

        return Math.abs(currentRPS - targetRPS) < VELOCITY_TOLERANCE_RPS;
    }

    public void stopShooter() {
        shooterMotor.stopMotor();
    }
    @Override
public void periodic() {
        double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/VelocityRPM", currentRPS * 60.0);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        SmartDashboard.putNumber("Shooter/DistanceMeters", getDistanceMeters());
    }
}
