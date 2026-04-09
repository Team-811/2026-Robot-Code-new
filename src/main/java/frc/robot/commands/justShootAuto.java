package frc.robot.commands;

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
 * Minimal autonomous routine that only aims and shoots.
 *
 * <p>Like {@link leftDriveAuto}, the final parallel group does not contain a deadline or timeout, so
 * it keeps running until autonomous mode ends or another mode interrupts it.
 */
public class justShootAuto extends SequentialCommandGroup {
  public justShootAuto(
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      LimelightShooter limelight,
      Indexer indexer,
      intakeForNow in) {
    addCommands(
        new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new closeNeo2(shooterN),
            new shooterLime(shooterK),
            new IndexSpin(indexer),
            new lowerIntake(in).withTimeout(1)));
  }
}
