package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

public class shooterCommand extends Command{
  
  shooterNeoVortex neoVortex1;
  public shooterCommand(shooterNeoVortex neoVortex1){
    this.neoVortex1 = neoVortex1;

    addRequirements(neoVortex1);
  }

  @Override
  public void initialize() {
    //yuh
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neoVortex1.ssssssswirly_whirly();
  
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    neoVortex1.ssssssswirly_whirly_stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
