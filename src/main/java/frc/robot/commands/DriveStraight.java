/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class DriveStraight extends Command {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private boolean fieldRelative;
  private double target; // how many more degrees to the right to turn
  private double maxVel; // max velocity, between 0 and kMaxSpeedMetersPerSecond in Constants 
  private double maxAccel; // max acceleration, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
  private long profileStartTime; // initial time (time of starting point)
  private double currDist;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private double angleInput, angleTarget;   // angleTarget is an absolute gyro angle
  private Translation2d startLocation;
  private SwerveModuleState[] desiredStates;
  private FileLog log;
  private boolean isOpenLoop;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system
 
  /**
   * Drives the robot straight.
   * @param target distance to travel, in meters (+ = forward, - = backward)
   * @param fieldRelative false = angle is relative to current robot facing,
   *   true = angle is an absolute field angle (0 = away from drive station)
   * @param angle angle to drive along when driving straight (+ = left, - = right)
   * @param maxVel max velocity in meters/second, between 0 and kMaxSpeedMetersPerSecond in Constants
   * @param maxAccel max acceleration in meters/second2, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param isOpenLoop true = feed-forward only for velocity control, false = PID feedback velocity control
   * @param driveTrain reference to the drive train subsystem
   * @param log
   */
  public DriveStraight(double target, boolean fieldRelative, double angle, double maxVel, double maxAccel, boolean regenerate, boolean isOpenLoop, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.fieldRelative = fieldRelative;
    angleInput = angle;
    this.regenerate = regenerate;
    this.isOpenLoop = isOpenLoop;
    this.fromShuffleboard = false;
    this.target = target;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, SwerveConstants.kMaxSpeedMetersPerSecond);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, SwerveConstants.kMaxAccelerationMetersPerSecondSquare);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }


  /**
   * Use this constructor when reading values from Shuffleboard
   * @param fieldRelative false = angle is relative to current robot facing,
   *   true = angle is an absolute field angle (0 = away from drive station)
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param isOpenLoop true = feed-forward only for velocity control, false = PID feedback velocity control
   * @param driveTrain reference to the drive train subsystem
   * @param log
   */
  public DriveStraight(boolean fieldRelative, boolean regenerate, boolean isOpenLoop, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.fieldRelative = fieldRelative;
    angleInput = 0;
    this.regenerate = regenerate;
    this.isOpenLoop = isOpenLoop;
    this.fromShuffleboard = true;
    this.target = 0;
    this.maxVel = 0.5 * SwerveConstants.kMaxSpeedMetersPerSecond;
    this.maxAccel = 0.5 * SwerveConstants.kMaxAccelerationMetersPerSecondSquare;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    if(SmartDashboard.getNumber("DriveStraight Manual Target Dist", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Target Dist", 2);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual Angle", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Angle", 0);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxVel", 0.5*SwerveConstants.kMaxSpeedMetersPerSecond);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxAccel", 0.5*SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("DriveStraight Manual Target Dist", 2);
      angleInput = SmartDashboard.getNumber("DriveStraight Manual Angle", 0);
      maxVel = SmartDashboard.getNumber("DriveStraight Manual MaxVel", SwerveConstants.kMaxSpeedMetersPerSecond);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, SwerveConstants.kMaxSpeedMetersPerSecond);
      maxAccel = SmartDashboard.getNumber("DriveStraight Manual MaxAccel", SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
    }

    // Calculate target angle
    if (fieldRelative) {
      angleTarget = angleInput - driveTrain.getGyroRotation();
    } else {
      angleTarget = angleInput;
    }

    // Initialize swerve states
    SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(angleTarget));
    desiredStates = new SwerveModuleState[] { state, state, state, state };

    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel); // initialize velocity and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    log.writeLog(false, "DriveStraight", "init", "Target", target, "Profile total time", tProfile.totalTime());
    
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    startLocation = driveTrain.getPose().getTranslation();
    
    driveTrain.setDriveModeCoast(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update data for this iteration
    long currProfileTime = System.currentTimeMillis();
    double timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    currDist = driveTrain.getPose().getTranslation().getDistance(startLocation);

    // Get next state from trapezoid profile
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);
    double targetVel = tStateNext.velocity;
    double targetAccel = tStateNext.acceleration;

    // Set wheel speeds
    // Note:  All 4 SwerveModuleStates in the desiredStates[] array point to the same SwerveModuleState object.
    // So, they will always have the same speed, even if we only update one of the elements of the array.
    desiredStates[0].speedMetersPerSecond = targetVel;
    desiredStates[1].speedMetersPerSecond = targetVel;
    desiredStates[2].speedMetersPerSecond = targetVel;
    desiredStates[3].speedMetersPerSecond = targetVel;
    driveTrain.setModuleStates(desiredStates, isOpenLoop); 
    
    // Read current module states for logging
    SwerveModuleState[] currentStates = driveTrain.getModuleStates();
    double linearVel = (Math.abs(currentStates[0].speedMetersPerSecond) + Math.abs(currentStates[1].speedMetersPerSecond) +
        Math.abs(currentStates[2].speedMetersPerSecond) + Math.abs(currentStates[3].speedMetersPerSecond))/4.0;

    log.writeLog(false, "DriveStraight", "profile", "angT", angleTarget,
      "posT", tStateNext.position, 
      "velT", targetVel, "accT", targetAccel,
      "posA", currDist, 
      "velA", linearVel,
      "velA-FL", currentStates[0].speedMetersPerSecond, 
      "velA-FR", currentStates[1].speedMetersPerSecond, 
      "velA-BL", currentStates[2].speedMetersPerSecond, 
      "velA-BR", currentStates[3].speedMetersPerSecond
    );

    if(regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currDist, linearVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "DriveStraight", "End");
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(target - currDist) < 0.0125) {
      accuracyCounter++;
      log.writeLog(false, "DriveStraight", "WithinTolerance", "Target Dist", target, "Actual Dist", currDist, "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}