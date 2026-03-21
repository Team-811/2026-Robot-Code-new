package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Autonomous routine that drives to mid-field, intakes during motion, then transitions into shooting.
 *
 * <p>There is an important command-grouping detail here:
 * <ul>
 *   <li>the five-second {@link ParallelDeadlineGroup} only contains {@code shooterLime}</li>
 *   <li>{@code closeNeo2} and {@code IndexSpin} are separate commands that run afterward</li>
 * </ul>
 *
 * <p>That means the final two commands are not limited by the deadline group and, because they never
 * finish on their own, the routine continues running until the autonomous period ends or the command
 * is interrupted.
 */
public class goToMidAuto extends SequentialCommandGroup {
  public goToMidAuto(
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      intakeForNow intake,
      Indexer indexer,
      Intake intakeSpin,
      LimelightShooter limelight) {

    addCommands(
        new lowerIntake(intake).withTimeout(5),
        new ParallelCommandGroup(
            new PathPlannerAuto("goToMidAuto"),
            new IntakeSpin(intakeSpin).withTimeout(2)),
        new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        new shooterLime(shooterK).withTimeout(2),
        new ParallelDeadlineGroup(new shooterLime(shooterK).withTimeout(5)),
        new closeNeo2(shooterN),
        new IndexSpin(indexer));
  }
}
