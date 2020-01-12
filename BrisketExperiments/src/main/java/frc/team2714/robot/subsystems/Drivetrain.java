/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2714.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2714.robot.Constants;
import frc.team2714.robot.util.DrivingController;
import frc.team2714.robot.util.Odometer;

public class Drivetrain extends SubsystemBase {

	private static double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
	private static double kTrackWidth = 3; // feet
	private static double kWheelRadius = 3.0/12; // feet
	private static int kShaftEncoderResolution = 2048; // counts per revolution bore encoder
	private static double positionChangePerRotation = 8.73; // Motor rotation per shaft rotation
	public static double kMaxVelocity = 13; // feet per second
	public static double kMaxAcceleration = 3; // Max Accel fet per second squared

	public static double ksVolts = 0.141; // Constant feedforward term for the robot drive.
	public static double kvVoltSecondsPerMeter = 2.26; // Velocity-proportional feedforward term for the robot drive
	public static double kaVoltSecondsSquaredPerMeter = 0.433; //Acceleration-proportional feedforward term for the robot

	private final double rpmToFeet = 0.003135; // Convert RPM to ft/s

	// Tuning parameter (b > 0) for which larger values make convergence more aggressive like a proportional term
	public static double kRamseteB = 2;

	// Tuning parameter (0 &lt; zeta &lt; 1) for which larger values provide more damping in response
	public static double kRamseteZeta = 0.7;

	public static double kPDrivePos = 34.5;
	public static double kDDrivePos = 15.9;

	public static double kPDriveVel = 4;

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
			new DifferentialDriveKinematics(Units.feetToMeters(kTrackWidth));

	private DifferentialDriveOdometry m_odometer;

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

	// Neo Encoders
	private CANEncoder leftNeoEncoder;
	private CANEncoder rightNeoEncoder;

	//NavX
	public AHRS navx;

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

	public Odometer odometer = new Odometer(0,0,0) {

		@Override
		public void updateEncodersAndHeading() {
			this.headingAngle = -navx.getYaw() + 90;
			if(this.headingAngle < 0) {
				this.headingAngle += 360;
			}

//			this.leftPos = leftShaftEncoder.getDistance();
//			this.rightPos = rightShaftEncoder.getDistance();
//
//			double leftVelocity = leftShaftEncoder.getRate();
//			double rightVelocity = rightShaftEncoder.getRate();

			this.leftPos = getLeftNeoDistance();
			this.rightPos = getRightNeoDistance();


			double leftVelocity = getLeftNeoVelocity();
			double rightVelocity = getRightNeoVelocity();

			this.currentAverageVelocity = (leftVelocity + rightVelocity) / 2;
		}

	};

	public DrivingController drivingController = new DrivingController(0.02) {

		/**
		 * Use output from odometer and pass into autonomous driving controller
		 */
		@Override
		public void updateVariables(){
			this.currentX = odometer.getCurrentX();
			this.currentY = odometer.getCurrentY();
			this.currentAngle = odometer.getHeadingAngle();
			this.currentAverageVelocity = odometer.getCurrentAverageVelocity();
		}

		/**
		 * Link autonomous driving controller to the drive train motor control
		 */
		@Override
		public void driveRobot(double power, double pivot) {
			closedLoopArcade(power, 0);
//			System.out.println("Power = " + power + " || Pivot = " + pivot);
//			SmartDashboard.putNumber("Drive Robot Power = ", power);
		}
	};


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

		lMotor0.enableVoltageCompensation(12.0);
		rMotor0.enableVoltageCompensation(12.0);

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

//		leftShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);
//		rightShaftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kShaftEncoderResolution);

		leftShaftEncoder.setDistancePerPulse(0.0007819);
		rightShaftEncoder.setDistancePerPulse(0.0007819);

		navx = new AHRS(SPI.Port.kMXP);
		navx.reset();
		navx.zeroYaw();
		new Rotation2d();
		m_odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-navx.getAngle()));
		new Rotation2d();
		m_odometer.resetPosition(new Pose2d(), Rotation2d.fromDegrees(-navx.getAngle()));

		leftNeoEncoder = lMotor0.getEncoder();
		rightNeoEncoder = rMotor0.getEncoder();


		leftNeoEncoder.setPosition(0);
		rightNeoEncoder.setPosition(0);

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
		return m_odometer.update(getAngle(), getLeftNeoDistance(), getRightNeoDistance());
	}

	/**
	 * @return Left velocity in ft/s approximately.
	 */
	public double getLeftNeoVelocity(){
		return leftNeoEncoder.getPosition() / positionChangePerRotation * 2 * Math.PI * Units.feetToMeters(kWheelRadius) / 60;
	}

	/**
	 * @return Right velocity in ft/s approximately.
	 */
	public double getRightNeoVelocity(){
		return rightNeoEncoder.getPosition() / positionChangePerRotation * 2 * Math.PI * Units.inchesToMeters(kWheelRadius) / 60;
	}

	/**
	 *
	 * @return returns left neo distance in meters
	 */
	public double getLeftNeoDistance(){
		return Units.feetToMeters((leftNeoEncoder.getPosition() / positionChangePerRotation) * (2 * Math.PI * kWheelRadius));
	}

	/**
	 * @return returns right neo distance in meters
	 */
	public double getRightNeoDistance(){
		return Units.feetToMeters((leftNeoEncoder.getPosition() / positionChangePerRotation)* (2 * Math.PI * kWheelRadius));
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
		lPidController.setReference(((Units.metersToFeet(leftVel) * positionChangePerRotation) / (2 * Math.PI * kWheelRadius)) * 60, ControlType.kVelocity);
		rPidController.setReference(-((Units.metersToFeet(rightVel) * positionChangePerRotation) / (2 * Math.PI * kWheelRadius)) * 60, ControlType.kVelocity);
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
		odometer.integratePosition();
		currentPose = updateOdometry();
		SmartDashboard.putNumber("Left NEO Encoder Speed Ft/s", Units.metersToFeet(getLeftNeoVelocity()));
		SmartDashboard.putNumber("Right NEO Encoder Speed Ft/s", Units.metersToFeet(getRightNeoVelocity()));

		SmartDashboard.putNumber("X Pose", Units.metersToFeet(currentPose.getTranslation().getX()));
		SmartDashboard.putNumber("Y Pose", Units.metersToFeet(currentPose.getTranslation().getY()));

		SmartDashboard.putNumber("X Pose Meters", (currentPose.getTranslation().getX()));
		SmartDashboard.putNumber("Y Pose Meters", (currentPose.getTranslation().getY()));

		SmartDashboard.putNumber("NavX Angle", m_odometer.getPoseMeters().getRotation().getDegrees());

		SmartDashboard.putNumber("Old NavX Angle", odometer.getHeadingAngle());
		SmartDashboard.putNumber("Old X Pose Feet", Units.metersToFeet(odometer.getCurrentX()));
		SmartDashboard.putNumber("Old Y Pose Feet", Units.metersToFeet(odometer.getCurrentY()));

		SmartDashboard.putNumber("Left Encoder", leftShaftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", rightShaftEncoder.getDistance());
//		System.out.println("X Pose: " + currentPose.getTranslation().getX() + " | Y Pose: " + currentPose.getTranslation().getY() +
//				"NavX Angle" + m_odometer.getPoseMeters().getRotation().getDegrees());
	}

	public void resetAll() {
		navx.reset();
		m_odometer.resetPosition(new Pose2d(), new Rotation2d().fromDegrees(-navx.getAngle()));
		lMotor0.getEncoder().setPosition(0);
		rMotor0.getEncoder().setPosition(0);
	}

	public void resetCustomPosition(double x, double y) {
		navx.reset();
		m_odometer.resetPosition(new Pose2d(x,y,new Rotation2d()), new Rotation2d().fromDegrees(-navx.getAngle()));
		lMotor0.getEncoder().setPosition(0);
		rMotor0.getEncoder().setPosition(0);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		SmartDashboard.putNumber("Ramsete Volts ", leftVolts);
		lMotor0.set(leftVolts/12.0);
		rMotor0.set(-rightVolts/12.0);
	}

	public void closedLoopArcade(double velocity, double pivot) {
		pivot = pivot * 2.5;
		closedLoopTank(velocity - pivot, velocity + pivot);
		// System.out.println("pivot " + pivot);
	}

	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		lPidController.setReference(leftVelocity / rpmToFeet, ControlType.kVelocity);
		rPidController.setReference(-rightVelocity / rpmToFeet, ControlType.kVelocity);
		// System.out.println("ls: " + leftVelocity / rpmToFeet + " rs: " + -rightVelocity / rpmToFeet);
	}

	public double calculateVisionDistanceFromTarget(){
		double height2 = 0;
		double height1 = 0;
		double initialCameraAngle = 0;
		double targetDeltaAngle = 0;

		return (height2 - height1) / Math.tan(initialCameraAngle + targetDeltaAngle);
	}

}
