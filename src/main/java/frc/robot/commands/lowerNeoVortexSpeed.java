package frc.robot.commands;

import frc.robot.subsystems.shooterNeoVortex;
import edu.wpi.first.wpilibj2.command.Command;

public class lowerNeoVortexSpeed extends Command{


    public lowerNeoVortexSpeed(){

    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterNeoVortex.lowerSpeed();
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
