/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2714.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTable;
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

public class Drivetrain extends SubsystemBase {

	private static double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
	private static double kTrackWidth = 3; // feet
	private static double kWheelRadius = 3.0/12; // feet
	private static int kShaftEncoderResolution = 8192; // counts per revolution bore encoder
	private static double positionChangePerRotation = 8.6190767288208; // Motor rotation per shaft rotation
	public static double kMaxVelocity = 13; // feet per second
	public static double kMaxAcceleration = 6; // Max Acceleration fet per second squared

	public static double ksVolts = 0.149; // Constant feedforward term for the robot drive.
	public static double kvVoltSecondsPerFeet = 0.683; // Velocity-proportional feedforward term for the robot drive
	public static double kaVoltSecondsSquaredPerFeet = 0.148; //Acceleration-proportional feedforward term for the robot

	// Tuning parameter (b > 0) for which larger values make convergence more aggressive like a proportional term
	public static double kRamseteB = 0.1;

	// Tuning parameter (0 &lt; zeta &lt; 1) for which larger values provide more damping in response
	public static double kRamseteZeta = 0;

	public static double kPDriveVel = 0;


	private double kMinOutput = -1;
	private double kMaxOutput = 1;

	//PID Constants
	private double kP = 4.8e-5;
	private double kI = 5.0e-7;
	private double kD = 0.0;
	private double kIS = 0.0;

	private double lKFF = 1.77e-4;
	private double rKFF = 1.78e-4;


	private DifferentialDriveKinematics m_kinematics =
			new DifferentialDriveKinematics(kTrackWidth);

	private DifferentialDriveOdometry m_odometer =
			new DifferentialDriveOdometry(m_kinematics, new Rotation2d());

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

	//PID Controller
	private CANPIDController lPidController;
	private CANPIDController rPidController;

	//NavX
	private AHRS navx;

	private DifferentialDrive differentialDrive;

	private static Drivetrain drivetrainInstance = null;

	/**
	 * Return only once instance of a drivetrain.
	 * @return an instance of the drivetrain.
	 */
	public static Drivetrain getInstance(){
		if (drivetrainInstance == null)
			drivetrainInstance = new Drivetrain();
		return drivetrainInstance;
	}


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

		lMotor0.enableVoltageCompensation(12.1);
		rMotor0.enableVoltageCompensation(12.1);

		lMotor0.setSmartCurrentLimit(50);
		lMotor1.setSmartCurrentLimit(50);
		lMotor2.setSmartCurrentLimit(50);

		rMotor0.setSmartCurrentLimit(50);
		rMotor1.setSmartCurrentLimit(50);
		rMotor2.setSmartCurrentLimit(50);

		differentialDrive = new DifferentialDrive(lMotor0, rMotor0);
		differentialDrive.setSafetyEnabled(false);

		leftShaftEncoder.reset();
		rightShaftEncoder.reset();

		lMotor0.getEncoder().setPosition(0);
		rMotor0.getEncoder().setPosition(0);

		leftShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);
		rightShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);

		// Setup up PID coefficients
		lPidController = lMotor0.getPIDController();
		lPidController.setP(kP);
		lPidController.setI(kI);
		lPidController.setD(kD);
		lPidController.setIZone(kIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController = rMotor0.getPIDController();
		rPidController.setP(kP);
		rPidController.setI(kI);
		rPidController.setD(kD);
		rPidController.setIZone(kIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);

		navx = new AHRS(SPI.Port.kMXP);
		navx.reset();
		navx.zeroYaw();
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
		return Rotation2d.fromDegrees(Math.IEEEremainder(-navx.getYaw(), 360));
	}

	/**
	 * Returns the current wheel speeds.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftNeoVelocity(), getRightNeoVelocity());
	}

	public DifferentialDriveKinematics getKinematics() {
		return m_kinematics;
	}

	/**
	 * Updates the field-relative position.
	 */
	public Pose2d updateOdometry() {
		return m_odometer.update(getAngle(), getCurrentSpeeds());
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

	public void setCurvatureDrive(double xSpeed, double ySpeed){
		differentialDrive.curvatureDrive(xSpeed, ySpeed, true);
	}

	public void setTankDrive(double leftVel, double rightVel) { differentialDrive.tankDrive(leftVel, rightVel); }

	public double getAverageVelocity(){
		return ((getLeftNeoVelocity() + getRightNeoVelocity()) / 2);
	}

	public void setClosedLoopTank(double leftVel, double rightVel){
	
		System.out.println("Left and Right : " + leftVel + " | " + -rightVel);
		System.out.println("ENDED PATH");
		lPidController.setReference(((leftVel * positionChangePerRotation) / (2 * Math.PI * kWheelRadius)) * 60, ControlType.kVelocity);
		rPidController.setReference(-((rightVel * positionChangePerRotation) / (2 * Math.PI * kWheelRadius)) * 60, ControlType.kVelocity);
	}

	public void stopAll(){
		System.out.println("STOPPING ALL");
		lMotor0.set(0);
		rMotor0.set(0);
	}




	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	@Override
	public void periodic() {
		currentPose = updateOdometry();

		System.out.println("NavX Angle" + getAngle().getDegrees());

		SmartDashboard.putNumber("Left NEO Encoder Speed Ft/s", getLeftNeoVelocity());
		SmartDashboard.putNumber("Right NEO Encoder Speed Ft/s", getRightNeoVelocity());

		SmartDashboard.putNumber("X Pose", currentPose.getTranslation().getX());
		SmartDashboard.putNumber("Y Pose", currentPose.getTranslation().getY());

		// SmartDashboard.putNumber("NavX Angle", getAngle().getDegrees());
		
		
	}
}
