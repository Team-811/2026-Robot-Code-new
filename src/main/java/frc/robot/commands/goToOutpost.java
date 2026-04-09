package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;

public class goToOutpost extends SequentialCommandGroup {
  public goToOutpost (
      CommandSwerveDrivetrain drivetrain,
      shooterNeoVortex shooterN,
      Shooter shooterK,
      intakeForNow intake,
      Indexer indexer,
      Intake intakeSpin,
      LimelightShooter limelight) {

       addCommands(
        new lowerIntake(intake).withTimeout(1), new WaitCommand(1),
            new PathPlannerAuto("goToOutpost"), new WaitCommand(1), new PathPlannerAuto("MuckMove"),
        new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        new shooterLime(shooterK).withTimeout(2),
        new ParallelDeadlineGroup(new shooterLime(shooterK).withTimeout(5),
        new closeNeo2(shooterN),
        new IndexSpin(indexer),new raiseIntake(intake).withTimeout(1)));
  }
}
