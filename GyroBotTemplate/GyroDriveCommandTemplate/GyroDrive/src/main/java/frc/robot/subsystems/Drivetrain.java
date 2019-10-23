/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

	// Drivetrain motors
	private CANSparkMax lMotor0;
	private CANSparkMax lMotor1;
	private CANSparkMax lMotor2;
	private CANSparkMax rMotor0;
	private CANSparkMax rMotor1;
	private CANSparkMax rMotor2;


	private DifferentialDrive differentialDrive;

	// Gearbox encoders
	private Encoder leftShaftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, CounterBase.EncodingType.k4X);
	private Encoder rightShaftEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			CounterBase.EncodingType.k4X);



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
	}

	public void setCurvatureDrive(double xSpeed, double ySpeed){
		differentialDrive.curvatureDrive(xSpeed, ySpeed, true);
	}



	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Encoder = " , leftShaftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder = " , rightShaftEncoder.getDistance());
	}
}
