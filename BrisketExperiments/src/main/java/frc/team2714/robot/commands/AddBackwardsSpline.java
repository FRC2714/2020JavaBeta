package frc.team2714.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2714.robot.subsystems.Drivetrain;

public class AddBackwardsSpline extends InstantCommand {

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
    public void execute() {
        addBackwardsSpline(0,0,270,2,4.5,0,270,2,3,4,0,0);
        System.out.println("GENERATED SPLINE");
        drivetrain.drivingController.getControlPath().stream().toString();
    }
}
