package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Autonomous routine that starts from the right bump side.
 *
 * <p>As with several other autos in this project, the final parallel group has no internal timeout,
 * so it continues until autonomous mode itself ends.
 */
public class fromRightBumpAuto extends SequentialCommandGroup {

  public fromRightBumpAuto(
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      LimelightShooter limelight,
      Indexer indexer,
      intakeForNow in) {
    addCommands(
        new PathPlannerAuto("fromRightBumpAuto"),
        new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new closeNeo2(shooterN),
            new shooterLime(shooterK),
            new IndexSpin(indexer),
            new lowerIntake(in).withTimeout(1)));
  }
}
