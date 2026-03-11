package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        new FaceAprilTag(drivetrain, limelight).withTimeout(2), new WaitCommand(1),
        new ParallelDeadlineGroup( new IndexSpin(indexer).withTimeout(5), new shooterLime(shooterK), new closeNeo2(shooterN))
        );
    }
}
