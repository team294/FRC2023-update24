package frc.robot.commands.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class RightPickup extends SequentialCommandGroup {
    public RightPickup(DriveTrain s_Swerve){
        
        Trajectory oneConeTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0.0, 0.0, new Rotation2d(0)),
                List.of(
                ),
                new Pose2d(-3.168, 0, new Rotation2d(Math.toRadians(Math.PI))),
                // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(-1, 1), new Translation2d(2, -1)),
                
                
                
                Constants.TrajectoryConstants.swerveTrajectoryConfig);

        var thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                oneConeTrajectory,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                thetaController,
                (states) -> s_Swerve.setModuleStates(states, false),
                s_Swerve);


        addCommands(
            //add in drive straight
            new InstantCommand(() -> s_Swerve.resetPose(oneConeTrajectory.getInitialPose())),
            //Add in scoring command here when implamented.
            swerveControllerCommand
        );
    }
}