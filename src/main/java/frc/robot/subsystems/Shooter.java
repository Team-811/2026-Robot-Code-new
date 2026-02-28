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

        distanceToRPM.put(1.0, -700.0);
        // distanceToRPM.put(2.5, -3500.0);
        distanceToRPM.put(2.0, -1800.0);
        distanceToRPM.put(3.0, -2500.0);
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