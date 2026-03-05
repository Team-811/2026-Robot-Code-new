package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterNeoVortex;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;


public class leftAuto extends SequentialCommandGroup {

    public leftAuto(CommandSwerveDrivetrain drivetrain,shooterNeoVortex shooterN,Shooter shooterK, intakeForNow intake,Indexer indexer,Intake intakeSpin,LimelightShooter limelight){
    addCommands(new lowerIntake(intake).withTimeout(5),
        new PathPlannerAuto("leftAuto"),
        // new PathPlannerAuto("leftAutoIntake"),
        new FaceAprilTag(drivetrain, limelight).withTimeout(2),
        new ParallelDeadlineGroup( new IndexSpin(indexer).withTimeout(3), new shooterLime(shooterK).withTimeout(2), new shooterCommand(shooterN).withTimeout(2))
        // new FaceAprilTag(drivetrain, limelight).withTimeout(1),
        // new ParallelRaceGroup(new IndexSpin(indexer), new shooterLime(shooterK),new shooterCommand(shooterN))
        );
    }
}
//  addCommands(
//         new ParallelDeadlineGroup(new PathPlannerAuto("leftAuto"),new IndexSpin(indexer).withTimeout(3), new shooterLime(shooterK).withTimeout(2), new shooterCommand(shooterN).withTimeout(2)),
//         new FaceAprilTag(drivetrain, limelight).withTimeout(1),
//         new ParallelDeadlineGroup(new PathPlannerAuto("leftAutoIntake"),new lowerIntake(intake).withTimeout(5),new IntakeSpin(intakeSpin).withTimeout(2)),
//         new FaceAprilTag(drivetrain, limelight).withTimeout(1),
//         new ParallelRaceGroup(new IndexSpin(indexer), new shooterLime(shooterK),new shooterCommand(shooterN))
//         );