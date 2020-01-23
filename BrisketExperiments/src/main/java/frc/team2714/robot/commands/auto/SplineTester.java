package frc.team2714.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2714.robot.commands.drivetrain.DriverControl;
import frc.team2714.robot.commands.drivetrain.trajectory.AddForwardPath;
import frc.team2714.robot.commands.drivetrain.trajectory.AddReversePath;
import frc.team2714.robot.subsystems.Drivetrain;

public class SplineTester extends SequentialCommandGroup {

    public SplineTester(Drivetrain drivetrain){
        addCommands(
                new SequentialCommandGroup(
                        new AddForwardPath(drivetrain,
                                0,0,90,5,
                                4.5,10,90,5,
                                5,6,0,0)
//                        new AddReversePath(drivetrain,
//                                3.5,6.5,90,3,
//                                0,0,90,3,
//                                3,5,0,0)
                )

//                new SequentialCommandGroup(
//                        new InstantCommand(() -> drivetrain.odometer.setOffset(-180)),
//                        new AddReversePath(drivetrain,
//                                0,0,270,3,
//                                0,6.5,270,3,
//                                7,6,0,0)
//                )
        );
    }

}
