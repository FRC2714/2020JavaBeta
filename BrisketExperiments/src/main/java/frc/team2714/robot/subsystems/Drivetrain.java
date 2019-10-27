/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2714.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2714.robot.Constants;
import frc.team2714.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

	private static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
	private static final double kTrackWidth = 3; // feet
	private static final double kWheelRadius = 3.0/12; // feet
	private static final int kShaftEncoderResolution = 8192; // counts per revolution bore encoder
	private static final double positionChangePerRotation = 8.6190767288208; //

	private final DifferentialDriveKinematics m_kinematics =
			new DifferentialDriveKinematics(kTrackWidth);

	private final DifferentialDriveOdometry m_odometry =
			new DifferentialDriveOdometry(m_kinematics);

	private Pose2d currentPose;

	// Drivetrain motors
	private CANSparkMax lMotor0;
	private CANSparkMax lMotor1;
	private CANSparkMax lMotor2;
	private CANSparkMax rMotor0;
	private CANSparkMax rMotor1;
	private CANSparkMax rMotor2;

	// Gearbox encoders
	private Encoder leftShaftEncoder = new Encoder(Constants.p_leftEncoderA, Constants.p_leftEncoderB, true, CounterBase.EncodingType.k4X);
	private Encoder rightShaftEncoder = new Encoder(Constants.p_rightEncoderA, Constants.p_rightEncoderB, true,
			CounterBase.EncodingType.k4X);

	//NavX
	AHRS navx = new AHRS(SPI.Port.kMXP);

	private DifferentialDrive differentialDrive;



	/**
	 * Creates a new Drivetrain.
	 */
	public Drivetrain() {
		lMotor0 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
		lMotor1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
		lMotor2 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
		rMotor0 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
		rMotor1 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
		rMotor2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

		lMotor1.follow(lMotor0);
		rMotor1.follow(rMotor0);
		lMotor2.follow(lMotor0);
		rMotor2.follow(rMotor0);

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		lMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		lMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

		rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

		differentialDrive = new DifferentialDrive(lMotor0, rMotor0);
		differentialDrive.setSafetyEnabled(false);

		leftShaftEncoder.reset();
		rightShaftEncoder.reset();

		lMotor0.getEncoder().setPosition(0);
		rMotor0.getEncoder().setPosition(0);

		leftShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);
		rightShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);

	}

	public void setCurvatureDrive(double xSpeed, double ySpeed){
		differentialDrive.curvatureDrive(xSpeed, ySpeed, true);
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getFakeAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(0);
	}

	public Rotation2d getAngle(){
		return Rotation2d.fromDegrees(Math.IEEEremainder(navx.getFusedHeading(), 360) * -1);
	}

	/**
	 * Returns the current wheel speeds.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftNeoVelocity(), getRightNeoVelocity());
	}

	/**
	 * Updates the field-relative position.
	 */
	public Pose2d updateOdometry() {
		return m_odometry.update(getFakeAngle(), getCurrentSpeeds());
	}

	/**
	 *
	 * @return Left velocity in ft/s approximately.
	 */
	public double getLeftNeoVelocity(){
		return ((lMotor0.getEncoder().getVelocity() / 60.0) * (2 * Math.PI * kWheelRadius)) / positionChangePerRotation;
	}

	/**
	 *
	 * @return Right velocity in ft/s approximately.
	 */
	public double getRightNeoVelocity(){
		return -((rMotor0.getEncoder().getVelocity() / 60.0) * (2 * Math.PI * kWheelRadius)) / positionChangePerRotation;
	}

	public Pose2d getCurrentPose(){
		return currentPose;
	}


	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	@Override
	public void periodic() {
		currentPose = updateOdometry();

		SmartDashboard.putNumber("Left NEO Encoder Speed Ft/s", getLeftNeoVelocity());
		SmartDashboard.putNumber("Right NEO Encoder Speed Ft/s", getRightNeoVelocity());

		SmartDashboard.putNumber("X Pose", currentPose.getTranslation().getX());
		SmartDashboard.putNumber("Y Pose", currentPose.getTranslation().getY());


		SmartDashboard.putNumber("Raw Joystick 1 = " , RobotContainer.driverStick.getRawAxis(1));
		SmartDashboard.putNumber("Raw Joystick 4 = " , RobotContainer.driverStick.getRawAxis(4));

	}
}
