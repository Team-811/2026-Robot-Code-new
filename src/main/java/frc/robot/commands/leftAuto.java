// package frc.robot.commands;

// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.shooterNeoVortex;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.intakeForNow;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.LimelightShooter;


// public class leftAuto extends SequentialCommandGroup {

//     public leftAuto(CommandSwerveDrivetrain drivetrain,shooterNeoVortex shooterN,Shooter shooterK, intakeForNow intake,Indexer indexer,Intake intakeSpin,LimelightShooter limelight){
//         addCommands(
//             new ParallelRaceGroup( new PathPlannerAuto("leftAuto"), new shooterLime(shooterK), new shooterCommand(shooterN)),
//             new ParallelCommandGroup(new PathPlannerAuto("leftAutoIntake"),new lowerIntake(intake),new IntakeSpin(intakeSpin),new IndexSpin(indexer)),
//             new FaceAprilTag(drivetrain, limelight),
//             new ParallelRaceGroup( new shooterLime(shooterK), new shooterCommand(shooterN),new IndexSpin(indexer))
//         );
//     }
// }