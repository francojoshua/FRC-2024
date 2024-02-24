// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {

	private final ArmSubsystem armSubsystem;
	private final boolean reverse;
		
	
  public ArmCommand(ArmSubsystem armsubsystem, boolean reverse) {
    this.armSubsystem =  armsubsystem;
	this.reverse = reverse;
	
	addRequirements(armsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	
	
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	armSubsystem.setarm(reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	armSubsystem.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
