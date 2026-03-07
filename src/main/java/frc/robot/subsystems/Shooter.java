package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Falcon-based shooter that maps Limelight distance estimates to RPM setpoints.
 * The lookup table is intentionally sparse; CTRE velocity closed-looping (with kV feedforward)
 * smooths the command so wheels hit the target speed instead of relying on open-loop percent outputs.
 * If distance lookup fails, the subsystem bails out by stopping so we do not fire at unknown speeds.
 */
public class Shooter extends SubsystemBase {

    private static final String CAN_BUS = "CANivore";
    private static final int SHOOTER_ID = 55;
    private static final double SUPPLY_LIMIT_AMPS = 50.0;
    private static final double STATOR_LIMIT_AMPS = 80.0;
    private static final double CLOSED_LOOP_RAMP_S = 0.1;
    private static final double VOLTAGE_COMP_SAT = 12.0;

    private final TalonFX shooterMotor = new TalonFX(SHOOTER_ID, CAN_BUS);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final String LIMELIGHT_NAME = "limelight-shooter";

    private static final double LIMELIGHT_HEIGHT = 0.80;
    private static final double TARGET_HEIGHT = 2.10;
    private static final double LIMELIGHT_ANGLE = Units.degreesToRadians(25);

    private static final double defaultSpeed = 1800;

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

        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(slot0)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_LIMIT_AMPS)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_LIMIT_AMPS)
                .withStatorCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs().withPeakForwardVoltage(VOLTAGE_COMP_SAT).withPeakReverseVoltage(-VOLTAGE_COMP_SAT))
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CLOSED_LOOP_RAMP_S));

        shooterMotor.getConfigurator().apply(cfg);
        shooterMotor.setInverted(false); // keep RPM table positive

        distanceToRPM.put(1.0, 750.0);
        distanceToRPM.put(2.0, 1800.0);
        distanceToRPM.put(3.0, 2500.0);
    }

    public void runShooterWithLimelight() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
        if (!hasTarget) {
            targetRPM = 0;
            stopShooter();
            return;
        }

        double distance = getDistanceMeters();
        double clampedDistance = MathUtil.clamp(
            distance,
            distanceToRPM.firstKey(),
            distanceToRPM.lastKey()
        );

        Double rpm = distanceToRPM.get(clampedDistance);
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
            // No tag seen: fall back to a conservative fixed RPM so the command can still spin up safely.
            targetRPM = defaultSpeed;
        }
        
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
    @Override
public void periodic() {
    double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
    SmartDashboard.putNumber("Shooter/VelocityRPM", currentRPS * 60.0);
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
}
}
