package frc.team2714.robot.commands.drivetrain.trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2714.robot.subsystems.Drivetrain;

public class AddReversePath extends CommandBase {

    private Drivetrain drivetrain;

    private double xInitial, yInitial, thetaInitial, lInitial, xFinal, yFinal, thetaFinal, lFinal, maxAcceleration, maxVelocity, startVelocity, endVelocity;

    public AddReversePath(Drivetrain drivetrain,
                          double xInitial, double yInitial, double thetaInitial, double lInitial,
                          double xFinal, double yFinal, double thetaFinal, double lFinal,
                          double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity){
        this.drivetrain = drivetrain;

        this.xInitial = xInitial;
        this.yInitial = yInitial;
        this.thetaInitial = thetaInitial;
        this.lInitial = lInitial;
        this.xFinal = xFinal;
        this.yFinal = yFinal;
        this.thetaFinal = thetaFinal;
        this.lFinal = lFinal;
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
    }

    @Override
    public void initialize() {
        addBackwardsSpline(xInitial, yInitial, thetaInitial, lInitial, xFinal, yFinal, thetaFinal, lFinal, maxAcceleration, maxVelocity, startVelocity, endVelocity);
        drivetrain.odometer.reset();
        drivetrain.navx.zeroYaw();
        drivetrain.resetAll();
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
        System.out.printf("Final X Position %.2f | Final Y Position %.2f \n", drivetrain.odometer.getCurrentX(), drivetrain.odometer.getCurrentY());
        System.out.println("---ENDED REVERSE PATH TRACKING---");
        drivetrain.closedLoopArcade(0,0);
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
}
