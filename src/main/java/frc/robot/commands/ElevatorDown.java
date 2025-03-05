package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command {
  Elevator elevator;

  public ElevatorDown(Elevator elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.goDownElevator();
  }

  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
