// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.HolonomicDriveControllerBCR;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.Translation2dBCR;
import frc.robot.utilities.TrapezoidProfileBCR;

public class DriveToPose extends Command {
  private final DriveTrain driveTrain;
  private final FileLog log;
  
  private final Timer timer = new Timer();
  private SwerveDriveKinematics kinematics;
  private HolonomicDriveControllerBCR controller;

  private double maxThetaErrorDegrees = TrajectoryConstants.maxThetaErrorDegrees;      
  private double maxPositionErrorMeters = TrajectoryConstants.maxPositionErrorMeters;   

  // Options to control how the goal is specified
  public enum GoalMode {
    pose, poseSupplier, angleRelative, angleAbsolute, shuffleboard
  }

  private final GoalMode goalMode;
  private final TrapezoidProfileBCR.Constraints trapProfileConstraints;
  private Supplier<Pose2d> goalSupplier;    // Supplier for goalPose
  private Rotation2d rotation;              // Rotation for goalPose
  private Pose2d initialPose, goalPose;     // Starting and destination robot pose (location and rotation) on the field
  private Translation2d initialTranslation;     // Starting robot translation on the field
  private Translation2d goalDirection;          // Unit vector pointing from initial pose to goal pose = direction of travel0
  private TrapezoidProfileBCR profile;      // Relative linear distance/speeds from initial pose to goal pose 

  private Translation2d curRobotTranslation;    // Current robot translation relative to initialTranslation

  /**
   * Drives the robot to the desired pose in field coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param goalPose target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
   public DriveToPose(Pose2d goalPose, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.goalPose = goalPose;
    goalMode = GoalMode.pose;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose in field coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param goalPose target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param maxVelMetersPerSecond max velocity to drive, in meters per second
   * @param maxAccelMetersPerSecondSquare max acceleration/deceleration, in meters per second squared
   * @param maxPositionErrorMeters tolerance for end position in meters
   * @param maxThetaErrorDegrees tolerance for end theta in degrees
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(Pose2d goalPose, double maxVelMetersPerSecond, double maxAccelMetersPerSecondSquare, double maxPositionErrorMeters, double maxThetaErrorDegrees, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.goalPose = goalPose;
    this.maxPositionErrorMeters = maxPositionErrorMeters;
    this.maxThetaErrorDegrees = maxThetaErrorDegrees;
    goalMode = GoalMode.pose;
    trapProfileConstraints = new TrapezoidProfileBCR.Constraints(
      MathUtil.clamp(maxVelMetersPerSecond, -SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullSpeedMetersPerSecond), 
      MathUtil.clamp(maxAccelMetersPerSecondSquare, -SwerveConstants.kFullAccelerationMetersPerSecondSquare, SwerveConstants.kFullAccelerationMetersPerSecondSquare)
    );

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose in field coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param goalPoseSupplier A function that supplies the target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(Supplier<Pose2d> goalPoseSupplier, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    goalSupplier = goalPoseSupplier;
    goalMode = GoalMode.poseSupplier;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }


   /**
   * Drives the robot to the desired pose in field coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param goalPoseSupplier A function that supplies the target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param maxPositionErrorMeters tolerance for end position in meters
   * @param maxThetaErrorDegrees tolerance for end theta in degrees
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(Supplier<Pose2d> goalPoseSupplier, double maxPositionErrorMeters, double maxThetaErrorDegrees, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.maxPositionErrorMeters = maxPositionErrorMeters;
    this.maxThetaErrorDegrees = maxThetaErrorDegrees;
    goalSupplier = goalPoseSupplier;
    goalMode = GoalMode.poseSupplier;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }

  /**
   * Rotates the robot to the specified rotation using an arbitrary angle without moving laterally.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param type CoordType, kRelative (turn relative to current angle) or kAbsolute (turn to field angle)
   * @param rotation rotation to turn to, in degrees (+=turn left, -=turn right).  For absolute rotation,
   * the 0 degrees is facing away from the driver station.
   * @param driveTrain DriveTrain subsytem
   * @param log log
   */
  public DriveToPose(CoordType type, double rotation, DriveTrain driveTrain, FileLog log ){
    this.driveTrain = driveTrain;
    this.log = log;
    this.rotation = Rotation2d.fromDegrees(rotation);
    if (type==CoordType.kRelative) {
      goalMode = GoalMode.angleRelative;
    } else {
      goalMode = GoalMode.angleAbsolute;
    }
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose based on numbers inputed in shuffleboard.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    goalMode = GoalMode.shuffleboard;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();

    if(SmartDashboard.getNumber("DriveToPose XPos meters", -9999) == -9999) {
      SmartDashboard.putNumber("DriveToPose XPos meters", 2);
    }
    if(SmartDashboard.getNumber("DriveToPose YPos meters", -9999) == -9999){
      SmartDashboard.putNumber("DriveToPose YPos meters", 2);
    }
    if(SmartDashboard.getNumber("DriveToPose Rot degrees", -9999) == -9999) {
      SmartDashboard.putNumber("DriveToPose Rot degrees", 0);
    }
  }

  /**
   * Common code between multiple constructors
   */
  private void constructorCommonCode() {
    addRequirements(driveTrain);

    // Define the swerve drive kinematics
    kinematics = Constants.DriveConstants.kDriveKinematics;

    // Define the controller for robot rotation
    ProfiledPIDController thetaController = new ProfiledPIDController(
        TrajectoryConstants.kPThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Define the holomic controller to controll driving
    controller = new HolonomicDriveControllerBCR(
      new PIDController(TrajectoryConstants.kPXController, 0, 0),
      new PIDController(TrajectoryConstants.kPYController, 0, 0),
      thetaController );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset timer and controllers
    timer.reset();
    timer.start();
    controller.reset();

    // Get the initial pose
    initialPose = driveTrain.getPose();
    initialTranslation = initialPose.getTranslation();
    curRobotTranslation = initialTranslation;

    // Get the goal pose
    switch (goalMode) {
      case pose:  // Goal pose directly specified
        break;
      case poseSupplier:  // using a supplier in the constructor
        goalPose = goalSupplier.get();
        break;
      case shuffleboard:  // using Shuffleboard
        double xPos = SmartDashboard.getNumber("DriveToPose XPos meters", 0);
        double yPos = SmartDashboard.getNumber("DriveToPose YPos meters", 0);
        Rotation2d angleTarget = Rotation2d.fromDegrees(SmartDashboard.getNumber("DriveToPose Rot degrees", 0));
        goalPose = new Pose2d(xPos, yPos, angleTarget);
        break;
      case angleAbsolute:  // absolute angle, keep robot position
        goalPose = new Pose2d(driveTrain.getPose().getTranslation(), rotation);
        break;
      case angleRelative:  // relative angle, keep robot position
        goalPose = driveTrain.getPose().plus(new Transform2d(new Translation2d(), rotation));
        break;
  }

    // Calculate the direction and distance of travel
    Translation2d trapezoidPath = goalPose.getTranslation().minus(initialTranslation);
    goalDirection = Translation2dBCR.normalize(trapezoidPath);
    double goalDistance = trapezoidPath.getNorm();
    
    // Get the initial velocity in the direction of travel
    ChassisSpeeds robotSpeed = driveTrain.getRobotSpeeds();
    double initialVelocity = robotSpeed.vxMetersPerSecond*goalDirection.getX() + robotSpeed.vyMetersPerSecond*goalDirection.getY();

    // Create the profile.  The profile is linear distance (along goalDirection) relative to the initial pose
    TrapezoidProfileBCR.State initialState = new TrapezoidProfileBCR.State(0, initialVelocity);
    TrapezoidProfileBCR.State goalState = new TrapezoidProfileBCR.State(goalDistance, 0);
    profile = new TrapezoidProfileBCR(trapProfileConstraints, goalState, initialState);

    log.writeLog(false, "DriveToPose", "Initialize", 
      "Time", timer.get(), 
      "Goal X", goalPose.getTranslation().getX(),
      "Goal Y", goalPose.getTranslation().getY(),
      "Goal rot", goalPose.getRotation().getDegrees(), 
      "Robot X", initialTranslation.getX(),
      "Robot Y", initialTranslation.getY(),
      "Robot rot", initialPose.getRotation().getDegrees(),
      "Profile time",profile.totalTime()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();

    // Current robot location, translation is relative to starting position, rotation is absolute field rotation
    curRobotTranslation = driveTrain.getPose().getTranslation().minus(initialTranslation);
    Pose2d robotPose = new Pose2d(curRobotTranslation, Rotation2d.fromDegrees(driveTrain.getGyroRotation()));

    // Calculate current desired pose and velocity from the Trapezoid profile, relative to starting position
    TrapezoidProfileBCR.State desiredState = profile.calculate(curTime);
    Pose2d desiredPose = new Pose2d( goalDirection.times(desiredState.position), goalDirection.getAngle());

    //fudge in some kA
    double desiredVelocityMetersPerSecond = desiredState.velocity + (desiredState.acceleration * SwerveConstants.kADriveToPose);

    Rotation2d desiredRotation = goalPose.getRotation();

    ChassisSpeeds targetChassisSpeeds =
        controller.calculate(robotPose, desiredPose, desiredVelocityMetersPerSecond, desiredRotation);
    var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

    driveTrain.setModuleStates(targetModuleStates, false);

    ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
    log.writeLog(false, "DriveToPose", "Execute", 
        "Time", timer.get(), 
        "Trap X", desiredPose.getTranslation().getX(),
        "Trap Y", desiredPose.getTranslation().getY(),
        "Trap Accel", desiredState.acceleration,
        "Trap Vel", desiredState.velocity,
        "Trap Vel w/kA", desiredVelocityMetersPerSecond,
        "Robot XVel", robotSpeeds.vxMetersPerSecond,
        "Robot Pos Err", driveTrain.getPose().getTranslation().minus(goalPose.getTranslation()).getNorm(),
        "Robot Th Err", MathBCR.angleMinus(driveTrain.getGyroRotation(), goalPose.getRotation().getDegrees()),
        "Trap VelAng", desiredPose.getRotation().getDegrees(),
        "Target rot", desiredRotation.getDegrees(), 
        "Robot X", curRobotTranslation.getX(),
        "Robot Y", curRobotTranslation.getY(),
        "Robot Vel", Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond),
        "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
        "Robot rot", robotPose.getRotation().getDegrees(),
        "Pitch", driveTrain.getGyroPitch()
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();

    if (!interrupted) {
      driveTrain.stopMotors();
    }

    log.writeLog(false, "DriveToPose", "End", "Interrupted", interrupted); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    var timeout = timer.hasElapsed(profile.totalTime()+3.0);
    if (timeout) {
      log.writeLog(false, "DriveToPose", "timeout"); 
    }

    var gyro = MathBCR.angleMinus(driveTrain.getGyroRotation(), goalPose.getRotation().getDegrees());
    var posError = driveTrain.getPose().getTranslation().minus(goalPose.getTranslation()).getNorm();
    
    var finished = timeout ||         // if we 3 seconds after the profile completed, then end even if we are not within tolerance 
      ( timer.hasElapsed(profile.totalTime())  && 
        ( Math.abs(gyro) <= maxThetaErrorDegrees ) &&
        ( posError  <= maxPositionErrorMeters) );

    if (finished) {
      log.writeLog(false, "DriveToPose", "finished", "gyroError", gyro, "posError", posError, "maxTheta",maxThetaErrorDegrees, "maxMeters", maxPositionErrorMeters, "timer",timer.get()); 
    }

    return finished;
  }

}
