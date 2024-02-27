// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommand extends Command {

	private final ArmSubsystem armSubsystem;

	private final double POSITION_UP = 160.0;
	private final double POSITION_DOWN = 0.0;
		
	
  public ArmCommand(ArmSubsystem armsubsystem) {
    this.armSubsystem =  armsubsystem;
	
	addRequirements(armsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		if (armSubsystem.getAbsoluteEncoderPosition() > (POSITION_UP - 10)) {
			armSubsystem.setPosition(POSITION_DOWN);	
		}
		else {
			armSubsystem.setPosition(POSITION_UP);
		}
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
   if (Math.abs(armSubsystem.getAbsoluteEncoderPosition() - 160.0) <= 0.030) {
      System.out.println("SetArmPosition Complete");
      return true;
    }
    else {
      return false;
    }
  }
}
