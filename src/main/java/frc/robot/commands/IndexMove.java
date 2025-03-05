package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class IndexMove extends Command {
  Index index;

  public IndexMove(Index index) {
    this.index = index;

    addRequirements(index);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    index.indexMove();
  }

  @Override
  public void end(boolean interrupted) {
    index.indexStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
