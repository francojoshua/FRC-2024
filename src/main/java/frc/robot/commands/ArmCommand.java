// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {

	private final ArmSubsystem armSubsystem;

	private final double position;
		
	
  public ArmCommand(ArmSubsystem armsubsystem, double position) {
    this.armSubsystem =  armsubsystem;
	this.position = position;
	
	addRequirements(armsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		armSubsystem.setPosition(position);
		System.out.println("Beginning SetArmPosition");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
	//armSubsystem.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (Math.abs(armSubsystem.getPosition() - position) <= 0.030) {
      System.out.println("SetArmPosition Complete");
      return true;
    }
    else {
      return false;
    }
  }
}
