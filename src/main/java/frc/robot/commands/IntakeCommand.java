// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

	private final IntakeSubsystem intakesubsystem;
		
	
  public IntakeCommand(IntakeSubsystem intakesubsystem) {
    this.intakesubsystem =  intakesubsystem;
	addRequirements(intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	intakesubsystem.setspeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	intakesubsystem.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
