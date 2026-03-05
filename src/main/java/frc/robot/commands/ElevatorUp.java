package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANElevatorSubsystem;

import static frc.robot.Constants.ElevatorConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorUp extends Command {
  /** Creates a new Intake. */

  CANElevatorSubsystem elevatorSubsystem;

  public ElevatorUp(CANElevatorSubsystem elevatorSystem) {
    addRequirements(elevatorSystem);
    this.elevatorSubsystem = elevatorSystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    elevatorSubsystem.setDesiredRotation(ELEVATOR_UPPER_ROTATION);
    System.out.println("ELEVATOR GOING UP!");
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
