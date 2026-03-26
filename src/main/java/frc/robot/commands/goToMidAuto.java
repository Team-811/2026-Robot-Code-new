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
 * <p>This routine now deliberately uses all three drive speed presets:
 * fast for the PathPlanner traversal, slow for AprilTag facing, then normal before the shooting
 * sequence so teleop does not inherit an aggressive auto speed mode.
 */
public class goToMidAuto extends SequentialCommandGroup {
  public goToMidAuto(
      RobotContainer robotContainer,
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
        new SetNormalDriveMode(robotContainer),
        new shooterLime(shooterK).withTimeout(2),
        new ParallelDeadlineGroup(new shooterLime(shooterK).withTimeout(5)),
        new closeNeo2(shooterN),
        new IndexSpin(indexer));
  }
}
