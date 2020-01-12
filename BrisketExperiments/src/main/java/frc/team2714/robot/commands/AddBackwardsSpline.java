package frc.team2714.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2714.robot.subsystems.Drivetrain;
import frc.team2714.robot.util.MotionPose;

import java.util.ArrayList;

public class AddBackwardsSpline extends CommandBase {

    private Drivetrain drivetrain;
    public AddBackwardsSpline(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    public void addBackwardsSpline(double xInitial, double yInitial, double thetaInitial, double lInitial,
                                   double xFinal, double yFinal, double thetaFinal, double lFinal, double maxAcceleration,
                                   double maxVelocity, double startVelocity, double endVelocity) {

        thetaInitial = Math.toRadians(thetaInitial);
        thetaFinal = Math.toRadians(thetaFinal);

        double x2 = lInitial * Math.cos(thetaInitial + Math.PI) + xInitial;
        double x3 = lFinal * Math.cos(thetaFinal) + xFinal;
        double y2 = lInitial * Math.sin(thetaInitial + Math.PI) + yInitial;
        double y3 = lFinal * Math.sin(thetaFinal) + yFinal;

        System.out.println("Backwards Spline Generating");

        drivetrain.drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
                maxAcceleration, maxVelocity, startVelocity, endVelocity, false);
    }

    @Override
    public void initialize() {
        addBackwardsSpline(0,0,270,2,0,4.5,270,2,2.5,3,0,0);
        drivetrain.odometer.reset();
        drivetrain.odometer.setOffset(-180);
        drivetrain.navx.zeroYaw();
        drivetrain.resetAll();
        System.out.println("RAN ONCE");
    }

    @Override
    public void execute() {
        drivetrain.drivingController.run();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.drivingController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED");
        drivetrain.closedLoopArcade(0,0);
    }
}
