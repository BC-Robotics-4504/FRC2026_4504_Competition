package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANIntakeArmSubsystem;

import static frc.robot.Constants.IntakeArmConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseIntake extends Command {
  /** Creates a new Intake. */

  CANIntakeArmSubsystem armSubsystem;

  public RaiseIntake(CANIntakeArmSubsystem armSystem) {
    addRequirements(armSystem);
    this.armSubsystem = armSystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    armSubsystem.setDesiredAngle(INTAKE_ARM_UPPER_ANGLE);
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
