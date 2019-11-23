/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2714.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team2714.robot.commands.DriverControl;
import frc.team2714.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import static frc.team2714.robot.subsystems.Drivetrain.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


	// The robot's subsystems and commands are defined here...
	private final Drivetrain drivetrain = Drivetrain.getInstance();

	public static Joystick driverStick = new Joystick(0);
	public DriverControl driverControl = new DriverControl(drivetrain);


	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * Joystick} or {@link XboxController}), and then calling passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		TrajectoryConfig config =
				new TrajectoryConfig(5, Drivetrain.kMaxAcceleration)
						.setKinematics(drivetrain.getKinematics());


		// An example trajectory to follow.  All units in meters.
		Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Translation2d(2, 0)
				),
				new Pose2d(4, 0, new Rotation2d(0)),
				// Pass config
				config
		);

		Trajectory quinticTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Pose2d(0, 0, new Rotation2d(0)),
						new Pose2d(6, 0, new Rotation2d(0)),
						new Pose2d(8, -2, new Rotation2d(-90)),
						new Pose2d(8, -8, new Rotation2d(-90))
				),
				// Pass config
				config
		);

		RamseteCommand ramseteCommand = new RamseteCommand(
				quinticTrajectory,
				drivetrain::getCurrentPose,
				new RamseteController(kRamseteB, kRamseteZeta),
				ksVolts,
				kvVoltSecondsPerFeet,
				kaVoltSecondsSquaredPerFeet,
				drivetrain.getKinematics(),
				drivetrain::getLeftNeoVelocity,
				drivetrain::getRightNeoVelocity,
				new PIDController(kPDriveVel, 0, 0),
				new PIDController(kPDriveVel, 0, 0),
				// CustomRamseteCommand passes volts to the callback, so we have to rescale here
//				(left, right) -> drivetrain.setTankDrive(left / 12., right / 12.),
				(left, right) -> drivetrain.setClosedLoopTank(left,right),
				drivetrain
		);

		CustomRamseteCommand ramseteOtherConstructor = new CustomRamseteCommand(
				quinticTrajectory,
				drivetrain::getCurrentPose,
				new RamseteController(kRamseteB, kRamseteZeta),
				drivetrain.getKinematics(),
				(left, right) -> drivetrain.setClosedLoopTank(left,right), //print out if this is actually working if unsure
				drivetrain
		);
//		System.out.println(quinticTrajectory);

		return ramseteOtherConstructor;
	}

	public Command getDriverControl(){
		return driverControl;
	}
}
