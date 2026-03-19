package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooterNeoVortex;
public class StartAtHubAuto extends SequentialCommandGroup{
     public StartAtHubAuto(CommandSwerveDrivetrain drivetrain,shooterNeoVortex shooterN,Shooter shooterK, LimelightShooter limelight,Indexer indexer){
        addCommands(new PathPlannerAuto("StartAtHubAuto"),  new FaceAprilTag(drivetrain, limelight).withTimeout(2),new WaitCommand(1),
        new ParallelCommandGroup(new closeShooter(shooterN), new shooterLime(shooterK), new IndexSpin(indexer)));
     }

}
