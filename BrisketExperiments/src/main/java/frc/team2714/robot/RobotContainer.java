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

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team2714.robot.commands.DriverControl;
import frc.team2714.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2714.robot.util.BBQRamseteCommand;

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
	private Drivetrain drivetrain = new Drivetrain();

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
//		drivetrain.resetCustomPosition(10,-4.5);

		TrajectoryConfig config =
				new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(Drivetrain.kMaxAcceleration))
						.setKinematics(drivetrain.getKinematics());



		// An example trajectory to follow.  All units in meters.
		Trajectory simpleSCurve = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d().fromDegrees(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5))
				),
				new Pose2d(Units.feetToMeters(10), Units.feetToMeters(-4.5), new Rotation2d().fromDegrees(0)),
				// Pass config
				config
		);

		Trajectory quinticTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)),
						new Pose2d(Units.feetToMeters(5),Units.feetToMeters(-5),new Rotation2d(-90)),
						new Pose2d(Units.feetToMeters(10),Units.feetToMeters(-10),new Rotation2d(0))
				),
				// Pass config
				config
		);

		RamseteCommand ramseteCommand = new RamseteCommand(
				simpleSCurve,
				drivetrain::getCurrentPose,
				new RamseteController(kRamseteB, kRamseteZeta),
				drivetrain.getKinematics(),
				drivetrain::setClosedLoopTank, //print out if this is actually working if unsure
				drivetrain
		);

		Trajectory square = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Pose2d(0, 0, new Rotation2d(0)),
						new Pose2d(7,0,new Rotation2d(0)),
						new Pose2d(10,4, Rotation2d.fromDegrees(90)),
						new Pose2d(10,8, Rotation2d.fromDegrees(90)),
						new Pose2d(7,10, new Rotation2d().fromDegrees(180)),
						new Pose2d(3,10, new Rotation2d().fromDegrees(180)),
						new Pose2d(0,7, new Rotation2d().fromDegrees(-90)),
						new Pose2d(0,0, new Rotation2d().fromDegrees(-90))
				),
				// Pass config
				config
		);

		return ramseteCommand.andThen(() -> drivetrain.stopAll());
	}

	public Command getDriverControl(){
		return driverControl;
	}
}
