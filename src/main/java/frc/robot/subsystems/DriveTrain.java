// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.FileLog;
import static frc.robot.Constants.Ports.*;

import static frc.robot.Constants.DriveConstants.*;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSlewRegion;
import frc.robot.utilities.*;

// Vision imports
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;


public class DriveTrain extends SubsystemBase implements Loggable {

  // File logging variables
  private FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles

  // variables for swerve modules
  private final SwerveModule swerveFrontLeft;
  private final SwerveModule swerveFrontRight;
  private final SwerveModule swerveBackLeft;
  private final SwerveModule swerveBackRight;
  
  // variables for gyro and gyro calibration
  private final AHRS ahrs;
  private double yawZero = 0.0;
  private double pitchZero = 0.0;

  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc

  // variable to store vision camera
  private PhotonCameraWrapper camera;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator; 
  private final Field2d field = new Field2d();    // Field to dispaly on Shuffleboard

  //Slew rate limiter
  private final Elevator elevator;
  // private boolean elevatorUpPriorIteration = false;       // Tracking for elevator position from prior iteration


  private final SlewRateLimiter filterY = new SlewRateLimiter(maxAccelerationRate);
  private final SlewRateLimiter filterYTier2 = new SlewRateLimiter(maxAccelerationRateY);

  //  tier 4 = slowest acceleration, tier 1 = fastest acceleration
  private final SlewRateLimiter filterXTier1 = new SlewRateLimiter(maxAccelerationRate);   // limiter in X direction when elevator is out
  private final SlewRateLimiter filterXTier2 = new SlewRateLimiter(maxAccelerationRateAtScoreMid);   // limiter in X direction when elevator is out
  private final SlewRateLimiter filterXTier3 = new SlewRateLimiter(maxAccelerationRateBetweenScoreMidAndHigh);   // limiter in X direction when elevator is out
  private final SlewRateLimiter filterXTier4 = new SlewRateLimiter(maxAccelerationRateWithElevatorUp);   // limiter in X direction when elevator is out
  private ElevatorSlewRegion elevatorPriorIteration = ElevatorSlewRegion.min;
  private ElevatorSlewRegion elevatorPriorIterationY = ElevatorSlewRegion.min;

  /**
   * Constructs the DriveTrain subsystem
   * @param log FileLog object for logging
   */
  public DriveTrain(Field fieldUtil, Elevator elevator, FileLog log) {
    this.log = log; // save reference to the fileLog
    logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
    this.camera = new PhotonCameraWrapper(fieldUtil, log, logRotationKey);
    this.elevator = elevator;

    // create swerve modules
    swerveFrontLeft = new SwerveModule( "FL",
      CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, 
      false, false, false, offsetAngleFrontLeftMotor, log);
    swerveFrontRight = new SwerveModule( "FR",
      CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANTurnEncoderFrontRight, 
      false, false, false, offsetAngleFrontRightMotor, log);
    swerveBackLeft = new SwerveModule( "BL",
      CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANTurnEncoderBackLeft, 
      false, false, false, offsetAngleBackLeftMotor, log);
    swerveBackRight = new SwerveModule( "BR",
      CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANTurnEncoderBackRight, 
      false, false, false, offsetAngleBackRightMotor, log);

    // configure navX gyro
    AHRS gyro = null;
		try {
      gyro = new AHRS(SerialPort.Port.kUSB);
      // gyro.zeroYaw();   // *** Do not zero the gyro hardware!  The hardware zeros asynchronously from this thread, so an immediate read-back of the gyro may not yet be zeroed.
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;

    // zero gyro and initialize angular velocity variables
    zeroGyro();
    prevAng = getGyroRaw();
    currAng = getGyroRaw();
    prevTime = System.currentTimeMillis();
    currTime = System.currentTimeMillis();
    lfRunningAvg.reset();

    

    // create and initialize odometery
    // Set initial location to 0,0.
    poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), 
       getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)) );
    SmartDashboard.putData("Field", field);
  }
  

  // ************ Gryo methods

  /**
   * Verifies if Gyro is still reading
   * @return true = gryo is connected to Rio
   */
  public boolean isGyroReading() {
    return ahrs.isConnected();
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * Angle from gyro is negated, so that + = left and - = right
   * @return raw gyro angle, in degrees.
   */
  public double getGyroRaw() {
    return -ahrs.getAngle();
  }

  /**
	 * @return double, gyro pitch from 180 to -180, in degrees (postitive is nose up, negative is nose down)
	 */
	public double getGyroPitchRaw() {
		return -ahrs.getPitch();
  }

  public void resetGyroPitch(){
    pitchZero = getGyroPitchRaw();
  }

  /**
	 * Zero the gyro position in software to the current angle.
	 */
	public void zeroGyro() {
    yawZero = getGyroRaw(); // set yawZero to gyro angle
    pitchZero = getGyroPitchRaw();  // set pitchZero to gyro pitch
  }
  
  /**
	 * Zero the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
  }

  /**
	 * @return double, gyro angle from 180 to -180, in degrees (postitive is left, negative is right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = MathBCR.normalizeAngle(angle);
		return angle;
  }

  /**
	 * @return double, gyro pitch from 180 to -180, in degrees (postitive is nose up, negative is nose down)
	 */
	public double getGyroPitch() {
		return getGyroPitchRaw() - pitchZero;
  }

  /**
   * @return gyro angular velocity (with some averaging to reduce noise), in degrees per second.
   * Positive is turning left, negative is turning right.
   */
  public double getAngularVelocity () {
    return angularVelocity;
  }

  /**
   * @return angular velocity from motor velocity readings (NOT from gyro)
   * Positive is turning left, negative is turning right.
   */
  // public double getAngularVelocityFromWheels () {
    // In the 2022 code, this was more accurate than the angular velocity from
    // the gyro.  This was used in the DriveTurnGyro code.  However, angular velocity
    // was easy to calculate from a west coast driveTrain.  How do we calculate this
    // from a swerve drive train?  Do we need this method?
  //   return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  // }


  // ************ Swerve drive methods

  /**
   * Configures the motors and encoders for every swerve module.
   * The swerve modules are automatically configured in the SwerveModule constructors.  So, this
   * method should not need to be called.
   * <p>However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the motors and encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   */
  public void configureSwerveModules(){
    swerveFrontLeft.configSwerveModule();
    swerveFrontRight.configSwerveModule();
    swerveBackLeft.configSwerveModule();
    swerveBackRight.configSwerveModule();
  }

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    swerveFrontLeft.setMotorModeCoast(setCoast);
    swerveFrontRight.setMotorModeCoast(setCoast);
    swerveBackLeft.setMotorModeCoast(setCoast);
    swerveBackRight.setMotorModeCoast(setCoast);
  }

  /**
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setDriveMotorsOutput(double percentOutput){
    swerveFrontLeft.setDriveMotorPercentOutput(percentOutput);
    swerveFrontRight.setDriveMotorPercentOutput(percentOutput);
    swerveBackLeft.setDriveMotorPercentOutput(percentOutput);
    swerveBackRight.setDriveMotorPercentOutput(percentOutput);
  }
  

  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setTurningMotorsOutput(double percentOutput){
    swerveFrontLeft.setTurnMotorPercentOutput(percentOutput);
    swerveFrontRight.setTurnMotorPercentOutput(percentOutput);
    swerveBackLeft.setTurnMotorPercentOutput(percentOutput);
    swerveBackRight.setTurnMotorPercentOutput(percentOutput);
  }


  /**
   * Stops all of the drive and turning motors
   */
  public void stopMotors() {
    swerveFrontLeft.stopMotors();
    swerveFrontRight.stopMotors();
    swerveBackLeft.stopMotors();
    swerveBackRight.stopMotors();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.          
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {

    

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    // Convert states to chassisspeeds
    ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates);
    
    // y slew rate limit chassisspeed
    double ySlewed = filterY.calculate(chassisSpeeds.vyMetersPerSecond);

    if (elevator.getElevatorPos() <= ElevatorSlewRegion.min.position) {
      // Elevator is down.  We can X-travel at full speed
      if (!(elevatorPriorIterationY == ElevatorSlewRegion.min)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterY.reset(getChassisSpeeds().vyMetersPerSecond);
      }
      elevatorPriorIterationY = ElevatorSlewRegion.min;
      
      ySlewed = filterY.calculate(chassisSpeeds.vyMetersPerSecond);
    } else {
      // Use low region for anything that's not min
      if (!(elevatorPriorIterationY == ElevatorSlewRegion.low)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterYTier2.reset(MathUtil.clamp(getChassisSpeeds().vyMetersPerSecond, -ElevatorSlewRegion.medium.velocity, ElevatorSlewRegion.medium.velocity));
      }
      elevatorPriorIterationY = ElevatorSlewRegion.low;
      
      ySlewed = filterYTier2.calculate(MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -ElevatorSlewRegion.medium.velocity, ElevatorSlewRegion.medium.velocity));
    }

    double xSlewed, omegaLimited;
  

    if (elevator.getElevatorPos() <= ElevatorSlewRegion.min.position) {
      // Elevator is down.  We can X-travel at full speed
      if (!(elevatorPriorIteration == ElevatorSlewRegion.min)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterXTier1.reset(getChassisSpeeds().vxMetersPerSecond);
      }
      elevatorPriorIteration = ElevatorSlewRegion.min;
      
      omegaLimited = chassisSpeeds.omegaRadiansPerSecond;
      xSlewed = filterXTier1.calculate(chassisSpeeds.vxMetersPerSecond);
    } else if (elevator.getElevatorPos() <= ElevatorSlewRegion.low.position) {
      if (!(elevatorPriorIteration == ElevatorSlewRegion.low)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterXTier2.reset(MathUtil.clamp(getChassisSpeeds().vxMetersPerSecond, -ElevatorSlewRegion.low.velocity, ElevatorSlewRegion.low.velocity));
      }
      elevatorPriorIteration = ElevatorSlewRegion.low;
      
      omegaLimited = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -ElevatorSlewRegion.low.rotationRate, ElevatorSlewRegion.low.rotationRate);
      xSlewed = filterXTier2.calculate(MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -ElevatorSlewRegion.low.velocity, ElevatorSlewRegion.low.velocity));
    } else if (elevator.getElevatorPos() <= ElevatorSlewRegion.medium.position) {
      if (!(elevatorPriorIteration == ElevatorSlewRegion.medium)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterXTier3.reset(MathUtil.clamp(getChassisSpeeds().vxMetersPerSecond, -ElevatorSlewRegion.medium.velocity, ElevatorSlewRegion.medium.velocity));
      }
      elevatorPriorIteration = ElevatorSlewRegion.medium;
      
      omegaLimited = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -ElevatorSlewRegion.medium.rotationRate, ElevatorSlewRegion.medium.rotationRate);
      xSlewed = filterXTier3.calculate(MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -ElevatorSlewRegion.medium.velocity, ElevatorSlewRegion.medium.velocity));
    } else {
      // Elevator is up.  X-travel slowly!
      if (!(elevatorPriorIteration == ElevatorSlewRegion.max)) {
        // Elevator was up but is now down.  Reset the fast slew rate limiter
        filterXTier4.reset(MathUtil.clamp(getChassisSpeeds().vxMetersPerSecond, -ElevatorSlewRegion.max.velocity, ElevatorSlewRegion.max.velocity));
      }
      elevatorPriorIteration = ElevatorSlewRegion.max;
      
      omegaLimited = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -ElevatorSlewRegion.max.rotationRate, ElevatorSlewRegion.max.rotationRate);
      xSlewed = filterXTier4.calculate(MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -ElevatorSlewRegion.max.velocity, ElevatorSlewRegion.max.velocity));
    }


    // convert back to swervem module states
    desiredStates = kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSlewed, ySlewed, omegaLimited), new Translation2d());
    
    swerveFrontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    swerveFrontRight.setDesiredState(desiredStates[1], isOpenLoop);
    swerveBackLeft.setDesiredState(desiredStates[2], isOpenLoop);
    swerveBackRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  // TODO Add version of setModuleStates with acceleration

  /**
   * Reads the current swerve ModuleStates.
   * @return The current module states, as measured by the encoders.  
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      swerveFrontLeft.getState(), swerveFrontRight.getState(),
      swerveBackLeft.getState(), swerveBackRight.getState()
    };
  }

  /**
   * Returns the speed of the chassis in X, Y, and theta <b>in the robot frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = forward)
   * <p> Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Reads the current swerve ModulePositions.
   * @return The current module positions, as measured by the encoders.  
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      swerveFrontLeft.getPosition(), swerveFrontRight.getPosition(),
      swerveBackLeft.getPosition(), swerveBackRight.getPosition()
    };
  }
  
  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param fieldRelative True = the provided x and y speeds are relative to the field. False = the provided x and y speeds are relative to the current facing of the robot.
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
    drive(xSpeed, ySpeed, rot, new Translation2d(), fieldRelative, isOpenLoop);
  }

  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and XSpeed, YSpeed = 0, then the robot will rotate around that corner.
   *     This feature may not work if fieldRelative = True (need to test).
   * @param fieldRelative True = the provided x and y speeds are relative to the field.
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   * False = the provided x and y speeds are relative to the current facing of the robot. 
   */
   public void drive(double xSpeed, double ySpeed, double rot, Translation2d centerOfRotationMeters, boolean fieldRelative, boolean isOpenLoop) {
    
    SwerveModuleState[] swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            centerOfRotationMeters);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  // ************ Odometry methods

  /**
   * Returns the currently-estimated position of the robot on the field
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose.  I.e. defines the robot's
   * position and orientation on the field.
   * This method also resets the gyro, which is required for the pose
   * to properly reset.
   *
   * @param pose The pose to which to set the pose estimator.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   */
  public void resetPose(Pose2d pose) {
    zeroGyroRotation(pose.getRotation().getDegrees());
    poseEstimator.resetPosition( Rotation2d.fromDegrees(getGyroRotation()),
    getModulePositions(), pose );
  }
  
  /**
   * Returns the speed of the robot in X, Y, and theta <b>in the field frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = away from our drivestation)
   * <p> Speed of the robot in the y direction, in meters per second (+ = left when looking from our drivestation)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    // Calculation from chassisSpeed to robotSpeed is just the inverse of .fromFieldRelativeSpeeds.
    // Call .fromFieldRelativeSpeeds with the negative of the robot angle to do this calculation.
    return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(-getGyroRotation()));
  }


  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
    camera.enableFastLogging(enabled);
  }

  /**
   * Checks if the CAN bus and gyro are working.  Sometimes, when the robot boots up, either the CAN bus or
   * the gyro don't initialize properly.  ROBOT CODE WILL NOT BE ABLE TO CONTROL MOTORS when this happens, so
   * always check this before starting a match!
   * @return true = something is not working.  false = CAN bus and gyro are both working.
   */
  public boolean canBusError() {
    return ((swerveFrontLeft.getDriveBusVoltage() < 7.0) || (swerveFrontLeft.getDriveTemp() < 5.0) || !isGyroReading());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // save current angle and time for calculating angVel
    currAng = getGyroRaw();
    currTime = System.currentTimeMillis();
 
    // calculate angVel in degrees per second
    angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );

    // update 
    updateOdometry();
    
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      // Update data on SmartDashboard
      field.setRobotPose(poseEstimator.getEstimatedPosition());
      ChassisSpeeds robotSpeeds = getRobotSpeeds();
      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive X Velocity", robotSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive Y Velocity", robotSpeeds.vyMetersPerSecond);
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", getGyroRotation());
      SmartDashboard.putNumber("Drive AngVel", angularVelocity);
      SmartDashboard.putNumber("Drive Pitch", getGyroPitch());
      
      // position from poseEstimator (helpful for autos)
      Pose2d pose = poseEstimator.getEstimatedPosition();
      SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
      SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());

      // Values from each swerve module
      swerveFrontLeft.updateShuffleboard();
      swerveFrontRight.updateShuffleboard();
      swerveBackLeft.updateShuffleboard();
      swerveBackRight.updateShuffleboard();

      // Values for bugfixing
      SmartDashboard.putNumber("Drive Bus Volt", swerveFrontLeft.getDriveBusVoltage());
    }

    // save current angVel values as previous values for next calculation
    prevAng = currAng;
    prevTime = currTime; 
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    log.writeLog(logWhenDisabled, "Drive", "Update Variables", 
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", angularVelocity, "Pitch", getGyroPitch(), 
      "Odometry X", pose.getTranslation().getX(), "Odometry Y", pose.getTranslation().getY(), 
      "Odometry Theta", pose.getRotation().getDegrees(),
      "Drive X Velocity", robotSpeeds.vxMetersPerSecond, 
      "Drive Y Velocity", robotSpeeds.vyMetersPerSecond,
      swerveFrontLeft.getLogString(),
      swerveFrontRight.getLogString(),
      swerveBackLeft.getLogString(),
      swerveBackRight.getLogString()
      );
  }

  public void updateOdometry() {
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions());

    // Only run camera updates for pose estimator in teleop mode
    if (camera.hasInit() && DriverStation.isTeleop()) {
      Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

      if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        // only updates odometry if close enough
        // TODO change how it decides if it's too far
        //if (camPose.estimatedPose.getX() < 3.3) {
          poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

          //field.getObject("Vision").setPose(camPose.estimatedPose.toPose2d());
          SmartDashboard.putNumber("Vision X", camPose.estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("Vision Y", camPose.estimatedPose.toPose2d().getY());
          SmartDashboard.putNumber("Vision rot", camPose.estimatedPose.toPose2d().getRotation().getDegrees());
          
          SmartDashboard.putNumber("Odo X", poseEstimator.getEstimatedPosition().getX());
          SmartDashboard.putNumber("Odo Y", poseEstimator.getEstimatedPosition().getY());
          SmartDashboard.putNumber("Odo rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
       // }
      }

    }

    if (camera.getAlliance() == Alliance.Red) {
      Pose2d currPose = poseEstimator.getEstimatedPosition();
      double x = FieldConstants.length - currPose.getX();
      double y = FieldConstants.width - currPose.getY();
      Rotation2d rot = currPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

      field.setRobotPose(new Pose2d(x, y, rot));
    } else field.setRobotPose(poseEstimator.getEstimatedPosition());
  }  

  public void cameraInit() {
    camera.init();
  }

}
