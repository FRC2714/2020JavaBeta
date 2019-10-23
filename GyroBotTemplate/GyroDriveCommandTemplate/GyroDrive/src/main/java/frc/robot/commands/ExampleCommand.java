/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {

	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final ExampleSubsystem m_subsystem;


	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ExampleCommand(ExampleSubsystem subsystem) {
		m_subsystem = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void execute() {
		if(Math.abs(m_subsystem.driverStick.getRawAxis(1)) > 0.035 && Math.abs(m_subsystem.driverStick.getRawAxis(4)) > 0.035)
			m_subsystem.setCurvatureDrive(-m_subsystem.driverStick.getRawAxis(1), m_subsystem.driverStick.getRawAxis(4));
		else
			m_subsystem.setCurvatureDrive(0,0);
	}

}
