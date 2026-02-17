package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterKraken;
import frc.robot.subsystems.shooterNeoVortex;

public class shooterCommand extends Command{
    shooterKraken kraken;
    shooterNeoVortex neoVortex1, neoVortex2;
    public shooterCommand(){
        neoVortex1 = new shooterNeoVortex(0);
        neoVortex2 = new shooterNeoVortex(0);
        kraken = new shooterKraken(0);
    }
    @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
