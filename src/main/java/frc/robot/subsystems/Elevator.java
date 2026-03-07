package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Elevator climber subsystem.
 * Design intent and rationale:
 * - Two NEO lift motors run together for symmetry; we drive them identically to reduce twist.
 * - One hook motor actuates the ladder hook to latch/unlatch.
 * - Control is open-loop percent output to stay simple; without sensors/limits we rely on conservative speeds
 *   and timed sequences. Speeds/timings live in ClimberConstants for quick tuning.
 * - No advanced SparkMax config is used here because the available API set is limited in this project; if
 *   your REVLib exposes current limits/voltage comp/idle mode, add them back for better protection.
 */
public class Elevator extends SubsystemBase {
    private final SparkMax liftLeft = new SparkMax(ClimberConstants.liftLeftNeoId, MotorType.kBrushless);
    private final SparkMax liftRight = new SparkMax(ClimberConstants.liftRightNeoId, MotorType.kBrushless);
    private final SparkMax hookMotor = new SparkMax(ClimberConstants.hookMotorId, MotorType.kBrushed);

    public Elevator() {
        // Basic initialization only. Default inversion is non-inverted; if a motor spins the wrong way,
        // flip the sign of the speed in ClimberConstants rather than using deprecated setInverted().
    }

    /** Raise the robot. Open-loop by design; keep speeds conservative to avoid brownouts/shock. */
    public void liftUp() {
        setLift(ClimberConstants.liftUpSpeed);
    }

    /** Lower the robot; slower to avoid slamming. */
    public void liftDown() {
        setLift(ClimberConstants.liftDownSpeed);
    }

    /** Stop lift motors. */
    public void stopLift() {
        setLift(0.0);
    }

    /** Close/engage hook to latch the rung. */
    public void hookClose() {
        hookMotor.set(ClimberConstants.hookCloseSpeed);
    }

    /** Open/disengage hook to move to the next rung. */
    public void hookOpen() {
        hookMotor.set(ClimberConstants.hookOpenSpeed);
    }

    /** Stop hook motor. */
    public void stopHook() {
        hookMotor.set(0.0);
    }

    private void setLift(double percent) {
        double clamped = Math.max(-1.0, Math.min(1.0, percent));
        liftLeft.set(clamped);
        liftRight.set(clamped);
    }
}
