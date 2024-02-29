// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intake;

public class IntakeCommand extends Command {

	private final IntakeSubsystem intakeSubsystem;
		
	private final boolean autoStop;
	
  public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean autoStop) {
    this.intakeSubsystem = intakeSubsystem;
	this.autoStop = autoStop;

	addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	intakeSubsystem.setspeed();
	Timer.delay(.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	intakeSubsystem.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoStop && intakeSubsystem.check_note();
  }
}
