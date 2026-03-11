package frc.robot.commands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;
public class goToMidAuto extends SequentialCommandGroup{
    public goToMidAuto(CommandSwerveDrivetrain drivetrain,shooterNeoVortex shooterN,Shooter shooterK, intakeForNow intake,Indexer indexer,Intake intakeSpin,LimelightShooter limelight){
        
    addCommands(new lowerIntake(intake).withTimeout(5),new ParallelCommandGroup(
        new PathPlannerAuto("goToMidAuto"), new IntakeSpin(intakeSpin)), new shooterLime(shooterK).withTimeout(2),
        new ParallelCommandGroup( new shooterLime(shooterK).withTimeout(5)), new closeNeo2(shooterN), new IndexSpin(indexer));
    }
}
