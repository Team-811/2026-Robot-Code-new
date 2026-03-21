package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Climber subsystem with two lift motors and one hook motor.
 *
 * <p>The naming in this project calls it {@code Elevator}, but functionally this class is the
 * climber mechanism used by {@link frc.robot.commands.ClimbCommand} and
 * {@link frc.robot.commands.DescendCommand}.
 *
 * <p>Design choices:
 * <ul>
 *   <li>two brushless lift motors run together to raise/lower the robot</li>
 *   <li>one brushed hook motor opens/closes the hook</li>
 *   <li>control is open-loop percent output, so command sequences rely on conservative timed steps</li>
 *   <li>most tuning lives in {@link frc.robot.Constants.ClimberConstants}</li>
 * </ul>
 *
 * <p>Because there are no active position limits or sensors in this subsystem, it is especially
 * important to tune speeds and timings carefully on the real mechanism.
 */
public class Elevator extends SubsystemBase {
    private final SparkMax liftLeft = new SparkMax(ClimberConstants.liftLeftNeoId, MotorType.kBrushless);
    private final SparkMax liftRight = new SparkMax(ClimberConstants.liftRightNeoId, MotorType.kBrushless);
    private final SparkMax hookMotor = new SparkMax(ClimberConstants.hookMotorId, MotorType.kBrushed);

    public Elevator() {
        // Basic initialization only. If a motor runs the wrong direction, change the sign of the
        // corresponding constant rather than patching every call site.
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
