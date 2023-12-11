package frc.robot.commands.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class CenterBalanceAuto extends SequentialCommandGroup {
    public CenterBalanceAuto(DriveTrain s_Swerve){
        
        Trajectory centerBalanceTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the position (1.75895, 2.707) facing +X direction
                new Pose2d(1.75895, 2.707, new Rotation2d(0)),
                List.of(),
                // Go straight onto platform
                new Pose2d(5.75895, 2.707, new Rotation2d(0)),
                Constants.TrajectoryConstants.swerveTrajectoryConfig);

        var thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                centerBalanceTrajectory,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                thetaController,
                (states) -> s_Swerve.setModuleStates(states, false),
                s_Swerve);


        addCommands(
            //add in drive straight
            new InstantCommand(() -> s_Swerve.resetPose(centerBalanceTrajectory.getInitialPose())),
            //Add in scoring command here when implamented.
            swerveControllerCommand
        );
    }
}