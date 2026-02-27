package frc.robot.commands;
import frc.robot.subsystems.shooterNeoVortex; 
import edu.wpi.first.wpilibj2.command.Command;

public class closeNeo2 extends Command{
     shooterNeoVortex neoVortex1;
    public closeNeo2(shooterNeoVortex neoVortex1){
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
    neoVortex1.close2();
  
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
