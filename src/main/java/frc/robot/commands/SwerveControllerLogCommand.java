// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.HolonomicDriveControllerBCR;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This command is copied from edu.wpi.first.wpilibj2.command.SwerveControllerCommand,
 * adding FileLog logging.
 * 
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class SwerveControllerLogCommand extends Command {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveControllerBCR m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final FileLog m_log;
  private final DriveTrain m_driveTrain;

  /**
   * This command is copied from edu.wpi.first.wpilibj2.command.SwerveControllerCommand,
   * adding FileLog logging.
   * 
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user to do since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param log FileLog for logging
   * @param requirements The subsystems to require.
   */
  public SwerveControllerLogCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      Consumer<SwerveModuleState[]> outputModuleStates,
      FileLog log,
      DriveTrain driveTrain) {
    this(
        trajectory,
        pose,
        kinematics,
        new HolonomicDriveControllerBCR(
            requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
            requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand")),
        desiredRotation,
        outputModuleStates,
        log,
        driveTrain);
  }

  /**
   * This command is copied from edu.wpi.first.wpilibj2.command.SwerveControllerCommand,
   * adding FileLog logging.
   * 
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param log FileLog for logging
   * @param requirements The subsystems to require.
   */
  public SwerveControllerLogCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      FileLog log,
      DriveTrain driveTrain) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        outputModuleStates,
        log,
        driveTrain);
  }

  /**
   * This command is copied from edu.wpi.first.wpilibj2.command.SwerveControllerCommand,
   * adding FileLog logging.
   * 
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param log FileLog for logging
   * @param requirements The subsystems to require.
   */
  public SwerveControllerLogCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveControllerBCR controller,
      Consumer<SwerveModuleState[]> outputModuleStates,
      FileLog log,
      DriveTrain driveTrain) {
    this(
        trajectory,
        pose,
        kinematics,
        controller,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        outputModuleStates,
        log,
        driveTrain);
  }

  /**
   * This command is copied from edu.wpi.first.wpilibj2.command.SwerveControllerCommand,
   * adding FileLog logging.
   * 
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param log FileLog for logging
   * @param requirements The subsystems to require.
   */
  public SwerveControllerLogCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveControllerBCR controller,
      Supplier<Rotation2d> desiredRotation,
      Consumer<SwerveModuleState[]> outputModuleStates,
      FileLog log,
      DriveTrain driveTrain) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");
    m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");

    m_desiredRotation =
        requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

    m_outputModuleStates =
        requireNonNullParam(outputModuleStates, "outputModuleStates", "SwerveControllerCommand");

    m_log = log;
    m_driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    m_controller.reset();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    Pose2d robotPose = m_pose.get();
    var desiredState = m_trajectory.sample(curTime);
    Rotation2d desiredRotation = m_desiredRotation.get();

    var targetChassisSpeeds =
        m_controller.calculate(robotPose, desiredState, desiredRotation);
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);

    ChassisSpeeds robotSpeeds = m_driveTrain.getRobotSpeeds();
    m_log.writeLog(false, "DriveTrajectory", "Execute", 
        "Time", m_timer.get(), 
        "Traj X", desiredState.poseMeters.getTranslation().getX(),
        "Traj Y", desiredState.poseMeters.getTranslation().getY(),
        "Traj Vel", desiredState.velocityMetersPerSecond,
        "Traj VelAng", desiredState.poseMeters.getRotation().getDegrees(),
        "Target rot", desiredRotation.getDegrees(), 
        "Robot X", robotPose.getTranslation().getX(),
        "Robot Y", robotPose.getTranslation().getY(),
        "Robot Vel", Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond),
        "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
        "Robot rot", robotPose.getRotation().getDegrees()
    );

  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()) &&
        ( Math.abs(m_pose.get().getRotation().getDegrees() - m_desiredRotation.get().getDegrees()) <= TrajectoryConstants.maxThetaErrorDegrees );
  }
}
