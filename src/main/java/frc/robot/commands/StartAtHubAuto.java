package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Autonomous routine that starts near the hub and then feeds a shot sequence.
 *
 * <p>The final parallel group has no deadline, so it stays active until auto mode ends or the
 * command is interrupted by a mode transition.
 */
public class StartAtHubAuto extends SequentialCommandGroup {
  public StartAtHubAuto(
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      LimelightShooter limelight,
      Indexer indexer) {
    addCommands(
        new PathPlannerAuto("StartAtHubAuto"),
        new FaceAprilTag(drivetrain, limelight).withTimeout(2),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new closeShooter(shooterN),
            new shooterLime(shooterK),
            new IndexSpin(indexer)));
  }
}
