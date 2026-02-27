package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeForNow;

public class raiseIntake extends Command{
    private intakeForNow in;
    public raiseIntake(intakeForNow in){
        this.in = in;
        addRequirements(in);
        
    }
     @Override
  public void initialize() {
    in.spinTheOtherWay();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    in.spinTheOtherWay();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    in.dontSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
