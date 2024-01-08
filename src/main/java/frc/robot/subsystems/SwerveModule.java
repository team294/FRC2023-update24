// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.CTREConfigs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.CTRESwerveModuleState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.Wait;

import static frc.robot.utilities.StringUtil.*;

public class SwerveModule {
      
  private final String swName;    // Name for this swerve module
  private final FileLog log;
  private final boolean cancoderReversed;
  private final double turningOffsetDegrees;

  // Drive motor objects
  private final TalonFX driveMotor;
	private final TalonFXConfigurator driveMotorConfigurator;
	private TalonFXConfiguration driveMotorConfig;
	private VoltageOut driveVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage driveVelocityControl = new VelocityVoltage(0.0);

  // Turning motor objects
  private final TalonFX turningMotor;
	private final TalonFXConfigurator turningMotorConfigurator;
	private TalonFXConfiguration turningMotorConfig;
	private VoltageOut turningVoltageControl = new VoltageOut(0.0);
  private PositionVoltage turningPositionControl = new PositionVoltage(0.0);

  // CANCoder objects
  private final WPI_CANCoder turningCanCoder;

  private double driveEncoderZero = 0;      // Reference raw encoder reading for drive FalconFX encoder.  Calibration sets this to zero.
  private double cancoderZero = 0;          // Reference raw encoder reading for CanCoder.  Calibration sets this to the absolute position from RobotPreferences.
  private double turningEncoderZero = 0;    // Reference raw encoder reading for turning FalconFX encoder.  Calibration sets this to match the CanCoder.

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSDrive, SwerveConstants.kVDrive, SwerveConstants.kADrive);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSTurn, SwerveConstants.kVTurn);
  

  /**
   * Constructs a SwerveModule.
   *
   * @param swName The name of this swerve module, for use in Shuffleboard and logging
   * @param driveMotorAddress The CANbus address of the drive motor.
   * @param turningMotorAddress The CANbus address of the turning motor.
   * @param cancoderAddress The CANbus address of the turning encoder.
   * @param cancoderReversed Whether the CANcoder is reversed.
   * @param turningOffsetDegrees Offset degrees in the turning motor to point to the 
   * front of the robot.  Value is the desired encoder zero point, in absolute magnet position reading.
   * @param log FileLog for logging
   */
  public SwerveModule(String swName, int driveMotorAddress, int turningMotorAddress, int cancoderAddress,
      boolean cancoderReversed, double turningOffsetDegrees, FileLog log) {

    // Save the module name and logfile
    this.swName = swName;
    this.log = log;
    this.cancoderReversed = cancoderReversed;
    this.turningOffsetDegrees = turningOffsetDegrees;

    // Create motor and encoder objects
    driveMotor = new TalonFX(driveMotorAddress);
    driveMotorConfigurator = driveMotor.getConfigurator();

    turningMotor = new TalonFX(turningMotorAddress);
    turningMotorConfigurator = turningMotor.getConfigurator();

    turningCanCoder = new WPI_CANCoder(cancoderAddress);

    // Configure the swerve module motors and encoders
    configSwerveModule();

    // other configs for drive and turning motors
    setMotorModeCoast(true);        // true on boot up, so robot is easy to push.  Change to false in autoinit or teleopinit

  }

  // ********** Swerve module configuration methods

  /**
   * Configures the motors and encoders, uses default values from the constructor.
   * In general, this should only be called from the constructor when the robot code is starting.
   * However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   * <p> <b>Note</b> that this procedure includes multiple blocking calls and will delay robot code.
   */
  public void configSwerveModule() {
    // **** configure drive motor

 		// Start with factory default TalonFX configuration
		driveMotorConfig = new TalonFXConfiguration();			// Factory default configuration
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // Supply current limit is typically used to prevent breakers from tripping.
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35.0;       // (amps) If current is above threshold value longer than threshold time, then limit current to this value
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;   // (amps) Threshold current
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;       // (sec) Threshold time
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // configure drive encoder
		driveMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Configure PID for VelocityVoltage control
    // Note:  In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    driveVelocityControl.Slot = 0;
    driveVelocityControl.OverrideBrakeDurNeutral = true;
    driveMotorConfig.Slot0.kP = 0.0;		// TODO calibrate.  2023 Phoenix5 = 0.10.  kP = (desired-output-volts) / (error-in-encoder-rps)
		driveMotorConfig.Slot0.kI = 0.0;
		driveMotorConfig.Slot0.kD = 0.0;    // TODO calibrate.  2023 Phoenix5 = 0.005.  kD = (desired-output-volts) / (error-in-encoder-rps/s)
		// driveMotorConfig.Slot0.kS = 0.0;
		// driveMotorConfig.Slot0.kV = 0.0;
		// driveMotorConfig.Slot0.kA = 0.0;

 		// Apply configuration to the drive motor.  
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		driveMotorConfigurator.apply(driveMotorConfig);

    // **** configure turning motor

 		// Start with factory default TalonFX configuration
		turningMotorConfig = new TalonFXConfiguration();			// Factory default configuration
    turningMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;		// Invert motor
		turningMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    turningMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
		turningMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Supply current limit is typically used to prevent breakers from tripping.
    turningMotorConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // (amps) If current is above threshold value longer than threshold time, then limit current to this value
    turningMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;   // (amps) Threshold current
    turningMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;       // (sec) Threshold time
    turningMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // configure drive encoder
		turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Configure PID for PositionVoltage control
    // Note:  In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    turningPositionControl.Slot = 0;
    turningPositionControl.OverrideBrakeDurNeutral = true;
    turningMotorConfig.Slot0.kP = 0.0;		// TODO calibrate.  2023 Phoenix5 = 0.15.  kP = (desired-output-volts) / (error-in-encoder-rotations)
		turningMotorConfig.Slot0.kI = 0.0;
		turningMotorConfig.Slot0.kD = 0.0;    // TODO calibrate.  2023 Phoenix5 = 3.0.  kD = (desired-output-volts) / (error-in-encoder-rps)
		// turningMotorConfig.Slot0.kS = 0.0;
		// turningMotorConfig.Slot0.kV = 0.0;
		// turningMotorConfig.Slot0.kA = 0.0;

 		// Apply configuration to the drive motor.  
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		turningMotorConfigurator.apply(turningMotorConfig);

    //TODO Stopped here!


    // **** configure turning CanCoder
    turningCanCoder.configFactoryDefault(100);
    turningCanCoder.configAllSettings(CTREConfigs.swerveCanCoderConfig, 100);
    turningCanCoder.configSensorDirection(cancoderReversed, 100);

    // NOTE!!! When the Cancoder or TalonFX encoder settings are changed above, then the next call to 
    // getCanCoderDegrees() getTurningEncoderDegrees() may contain an old value, not the value based on 
    // the updated configuration settings above!!!!  The CANBus runs asynchronously from this code, so 
    // sending the updated configuration to the CanCoder/Falcon and then receiving an updated position measurement back
    // may take longer than this code.
    // The timeouts in the configuration code above (100ms) should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(200);

    // System.out.println(swName + " CanCoder " + getCanCoderDegrees() + " FX " + getTurningEncoderDegrees() + " pre-CAN");
    zeroDriveEncoder();
    // log.writeLogEcho(true, "SwerveModule", swName+" pre-CAN", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
    calibrateCanCoderDegrees(turningOffsetDegrees);
    // log.writeLogEcho(true, "SwerveModule", swName+" post-CAN", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
    calibrateTurningEncoderDegrees(getCanCoderDegrees());
    // log.writeLogEcho(true, "SwerveModule", swName+" post-FX", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
  }

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    if (setCoast) {
      driveMotor.setNeutralMode(NeutralMode.Coast);
      turningMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralMode.Brake);
      turningMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  // ********** Main swerve module control methods

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveEncoderVelocity(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderMeters(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Turns off the drive and turning motors.
   */
  public void stopMotors() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setDriveMotorPercentOutput(double percentOutput){
    driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setTurnMotorPercentOutput(double percentOutput){
    turningMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * Sets the desired state for the module, using closed loop controls on the Talons.
   * The Talons will hold this state until commanded to stop or another state.
   * @param desiredState Desired state with speed and angle
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState =
        CTRESwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderDegrees()));

    // Set drive motor velocity or percent output
    if(isOpenLoop){
      driveMotor.set(ControlMode.PercentOutput, driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    else {
      driveMotor.set(ControlMode.Velocity, calculateDriveEncoderVelocityRaw(desiredState.speedMetersPerSecond), 
        DemandType.ArbitraryFeedForward, driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    // Set turning motor target angle
    // TODO Determine the right way to implement this code.  Make it selectable by a boolean parameter to setDesiredState?
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    // double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeedMetersPerSecond * 0.01)) 
    //   ? getTurningEncoderDegrees() : desiredState.angle.getDegrees(); 
    turningMotor.set(ControlMode.Position, calculateTurningEncoderTargetRaw(desiredState.angle.getDegrees())); 
  }

  // ********** Encoder methods

  // ******* Drive encoder methods

  /**
   * @return drive encoder position, in ticks
   */
  public double getDriveEncoderRaw() {
    return driveMotor.getSelectedSensorPosition(0);
  }

  /**
	 * Set the drive encoder position to zero in software.
	 */
  public void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRaw();
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "ZeroDriveEncoder", "driveEncoderZero", driveEncoderZero, "raw encoder", getDriveEncoderRaw(), "encoder meters", getDriveEncoderMeters());
  }

  /**
   * @return drive wheel distance travelled, in meters (+ = forward)
   */
  public double getDriveEncoderMeters() {
    return (getDriveEncoderRaw() - driveEncoderZero) * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * @return drive wheel velocity, in meters per second (+ = forward)
   */
  public double getDriveEncoderVelocity() {
    return driveMotor.getSelectedSensorVelocity(0) * SwerveConstants.kDriveEncoderMetersPerTick * 10.0;
  }

  /**
   * Converts a target velocity (in meters per second) to a target raw drive motor velocity.
   * @param velocityMPS Desired drive wheel velocity, in meters per second (+ = forward)
   * @return drive motor raw value equivalent velocity
   */
  public double calculateDriveEncoderVelocityRaw(double velocityMPS) {
    return velocityMPS / SwerveConstants.kDriveEncoderMetersPerTick / 10.0;
  }
  
  // ******* Turning TalonFX encoder methods

  /**
   * @return turning TalonFx encoder position, in ticks
   */
  public double getTurningEncoderRaw() {
    return turningMotor.getSelectedSensorPosition(0);
  }
  
  /**
   * Calibrates the turning FalconFX encoder.  Sets the current wheel facing to currentAngleDegrees.
   * @param currentAngleDegrees current angle, in degrees.
   */
  public void calibrateTurningEncoderDegrees(double currentAngleDegrees) {
    turningEncoderZero = getTurningEncoderRaw() - (currentAngleDegrees / SwerveConstants.kTurningEncoderDegreesPerTick);
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "calibrateTurningEncoder", "turningEncoderZero", turningEncoderZero, "raw encoder", getTurningEncoderRaw(), "set degrees", currentAngleDegrees, "encoder degrees", getTurningEncoderDegrees());
  }

  /**
   * @return turning FalconFX encoder facing, in degrees.  Values do not wrap, so angle could be greater than 360 degrees.
   * When calibrated, 0 should be with the wheel pointing toward the front of robot.
   * + = counterclockwise, - = clockwise
   */
  public double getTurningEncoderDegrees() {
    return (getTurningEncoderRaw() - turningEncoderZero) * SwerveConstants.kTurningEncoderDegreesPerTick;
  }

  /**
   * Converts a target wheel facing (in degrees) to a target raw turning FalconFX encoder value.
   * @param targetDegrees Desired wheel facing, in degrees.  0 = towards front of robot, + = counterclockwise, - = clockwise
   * @return turning FalconFX encoder raw value equivalent to input facing.
   */
  public double calculateTurningEncoderTargetRaw(double targetDegrees) {
    return targetDegrees / SwerveConstants.kTurningEncoderDegreesPerTick + turningEncoderZero;
  }

  /**
   * @return turning TalonFX encoder rotational velocity for wheel facing, in degrees per second.
   * + = counterclockwise, - = clockwise
   */
  public double getTurningEncoderVelocityDPS() {
    return turningMotor.getSelectedSensorVelocity(0) * SwerveConstants.kTurningEncoderDegreesPerTick * 10.0;
  }

  // ******* Cancoder methods

  /**
   * Calibrates the CanCoder encoder, so that 0 should be with the wheel pointing toward the front of robot.
   * @param offsetDegrees Desired encoder zero point, in absolute magnet position reading
   */
  public void calibrateCanCoderDegrees(double offsetDegrees) {
    // System.out.println(swName + " " + turningOffsetDegrees);
    // turningCanCoder.configMagnetOffset(offsetDegrees, 100);
    cancoderZero = -offsetDegrees;
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "calibrateCanCoder", "cancoderZero", cancoderZero, "raw encoder", turningCanCoder.getAbsolutePosition(), "encoder degrees", getCanCoderDegrees());
  }

  /**
   * @return turning CanCoder wheel facing, in degrees [-180,+180).
   * When calibrated, 0 should be with the wheel pointing toward the front of robot.
   * + = counterclockwise, - = clockwise
   */
  public double getCanCoderDegrees() {
    return MathBCR.normalizeAngle(turningCanCoder.getAbsolutePosition() - cancoderZero);
  }

  /**
   * @return turning CanCoder rotational velocity for wheel facing, in degrees per second.
   * + = counterclockwise, - = clockwise
   */
  public double getCanCoderVelocityDPS() {
    return turningCanCoder.getVelocity();
  }


  // ********** Information methods

  public double getDriveBusVoltage() {
    return driveMotor.getBusVoltage();
  }

  public double getDriveOutputVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  public double getDriveOutputPercent() {
    return driveMotor.getMotorOutputPercent();
  }

  public double getDriveStatorCurrent() {
    return driveMotor.getStatorCurrent();
  }

  public double getDriveTemp() {
    return driveMotor.getTemperature();
  }

  public double getTurningOutputVoltage() {
    return turningMotor.getMotorOutputVoltage();
  }

  public double getTurningOutputPercent() {
    return turningMotor.getMotorOutputPercent();
  }

  public double getTurningStatorCurrent() {
    return turningMotor.getStatorCurrent();
  }

  public double getTurningTemp() {
    return turningMotor.getTemperature();
  }

  /**
   * Updates relevant variables on Shuffleboard
   */
  public void updateShuffleboard() {
    SmartDashboard.putNumber(buildString("Swerve FXangle ", swName), MathBCR.normalizeAngle(getTurningEncoderDegrees()));
    SmartDashboard.putNumber(buildString("Swerve CCangle ", swName), getCanCoderDegrees());
    SmartDashboard.putNumber(buildString("Swerve FXangle dps", swName), getTurningEncoderVelocityDPS());
    SmartDashboard.putNumber(buildString("Swerve distance", swName), getDriveEncoderMeters());
    SmartDashboard.putNumber(buildString("Swerve drive temp ", swName), getDriveTemp());
  }

  /**
   * Returns information about the swerve module to include in the filelog
   * Format of the return string is comma-delimited name-value pairs, 
   * *without* the final comma.  Ex.  "name1,value1,name2,value2"
   */
  public String getLogString() {
    return buildString(
      swName, " CCangle deg,", getCanCoderDegrees(), ",",
      swName, " CCangle DPS,", getCanCoderVelocityDPS(), ",",
      swName, " FXangle deg,", MathBCR.normalizeAngle(getTurningEncoderDegrees()), ",",
      swName, " FXangle DPS,", getTurningEncoderVelocityDPS(), ",",
      swName, " turn output,", getTurningOutputPercent(), ",",
      swName, " drive meters,", getDriveEncoderMeters(), ",",
      swName, " drive mps,", getDriveEncoderVelocity(), ",",
      swName, " drive output,", getDriveOutputPercent(), ",",
      swName, " drive temp,", getDriveTemp(), ",",
      swName, " turn temp,", getTurningTemp()
    );
  }
}
