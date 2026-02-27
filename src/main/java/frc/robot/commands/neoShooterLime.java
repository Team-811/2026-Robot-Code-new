// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.neoShooterWithLime;

// public class neoShooterLime extends Command {

//     private final neoShooterWithLime shooter;

//     public neoShooterLime(neoShooterWithLime shooter) {
//         this.shooter = shooter;
//         addRequirements(shooter);
//     }

//     @Override
//     public void execute() {
//         shooter.runShooterWithLimelight();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooter.stopShooter();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }