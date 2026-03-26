package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Autonomous routine for the left starting position.
 *
 * <p>Sequence as written:
 * <ol>
 *   <li>lower the intake arm for up to 5 seconds</li>
 *   <li>run the PathPlanner auto named {@code leftAuto}</li>
 *   <li>briefly face the selected AprilTag</li>
 *   <li>wait one second for the robot to settle</li>
 *   <li>run indexer + both shooter subsystems, ending after the 5-second indexer timeout</li>
 * </ol>
 */
public class leftAuto extends SequentialCommandGroup {

  public leftAuto(
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      intakeForNow intake,
      Indexer indexer,
      Intake intakeSpin,
      LimelightShooter limelight) {
    addCommands(
        new lowerIntake(intake).withTimeout(1),
        new PathPlannerAuto("leftAuto"),
        new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        new WaitCommand(1),
        new ParallelDeadlineGroup(
            new IndexSpin(indexer).withTimeout(5),
            new shooterLime(shooterK),
            new closeShooter(shooterN)));
  }
}
