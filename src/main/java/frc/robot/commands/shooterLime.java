package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class shooterLime extends Command{ 

    private final Shooter shooter;

    public shooterLime(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.runShooterWithLimelight();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
