// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleOpCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController controller =
			new CommandXboxController(OperatorConstants.kDriverControllerPort);

	private final Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);
	

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		swerveSubsystem.setDefaultCommand(
				new SwerveTeleOpCommand(swerveSubsystem, () -> controller.getLeftY(),
						() -> controller.getLeftX(), () -> controller.getRightX(), () -> joystick.getRawButton(ControllerConstants.LY_ID)));


		
		configureBindings();
		configureAuto();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(m_exampleSubsystem::exampleCondition)
		// .onTrue(new ExampleCommand(m_exampleSubsystem));
		

		controller.a().onTrue(Commands.runOnce(swerveSubsystem::zeroHeading));

		controller.leftBumper().onTrue(new ArmCommand(armSubsystem, ArmConstants.kArmUpPosition));
		controller.leftTrigger().onTrue(new ArmCommand(armSubsystem, ArmConstants.kArmDownPosition));


		controller.rightTrigger().toggleOnTrue(new IntakeCommand(intake, true));
		controller.b().onTrue(new IntakeCommand(intake, false).withTimeout(1.5));
		controller.y().onTrue(Commands.runEnd(() -> intake.setspeed(-0.1), () -> intake.stopmotors()).withTimeout(0.1));


		controller.rightBumper().onTrue(Commands.runOnce(swerveSubsystem::toggleSlowMode));



		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	public void configureAuto() {
		NamedCommands.registerCommand("RunArm", new ArmCommand(armSubsystem, ArmConstants.kArmUpPosition).withTimeout(1.0));
		NamedCommands.registerCommand("RunIntake", new IntakeCommand(intake, false).withTimeout(1.5));
		NamedCommands.registerCommand("SlowOff", new InstantCommand(() -> swerveSubsystem.disableSlowMode()));

	
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return new SequentialCommandGroup(
			new InstantCommand(() -> swerveSubsystem.resetPose(PathPlannerPath.fromPathFile("Test").getPreviewStartingHolonomicPose())),
			new PathPlannerAuto("ScoreAmp")
		);
	}
}
