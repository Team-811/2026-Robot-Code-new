package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooterNeoVortex;

public class reverseShooter extends Command{
   shooterNeoVortex shooter;
    public reverseShooter(shooterNeoVortex shooter){
     
        this.shooter = shooter;

    addRequirements(shooter);
    }
      @Override
  public void initialize() {
    //yuh
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     shooter.spinTheOtherWay();
  
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     shooter.ssssssswirly_whirly_stop();    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
