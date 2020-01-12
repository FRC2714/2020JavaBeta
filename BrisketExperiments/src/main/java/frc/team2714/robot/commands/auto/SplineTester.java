package frc.team2714.robot.commands.auto;

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
                                0,0,90,3,
                                0,6.5,90,3,
                                7,6,0,0)

                )
        );
    }

}
