package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;
public class raiseNeoVortexSpeed extends Command{
    public raiseNeoVortexSpeed(){

    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterNeoVortex.raiseSpeed();
    }
 
  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
