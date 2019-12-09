/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2714.robot.commands;

import frc.team2714.robot.RobotContainer;
import frc.team2714.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriverControl extends CommandBase {

	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Drivetrain drivetrain;



	/**
	 * Creates a new DriverControl.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DriverControl(Drivetrain subsystem) {
		drivetrain = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void execute() {
		double rawX = RobotContainer.driverStick.getRawAxis(1);
		double rawPivot = RobotContainer.driverStick.getRawAxis(4);

		if(Math.abs(rawX) < 0.07)
			rawX = 0;

		if(Math.abs(rawPivot) < 0.1)
			rawPivot = 0;

		drivetrain.setCurvatureDrive(-rawX, rawPivot);
	}

}
