package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
public class IntakeSpin extends Command {

    Intake spinney;
    public IntakeSpin(Intake spinney){
        this.spinney = spinney;
        addRequirements(spinney);
    }
 @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinney.spin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinney.dontspin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
