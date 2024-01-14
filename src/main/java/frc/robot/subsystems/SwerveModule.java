// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

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

	// Drive motor signals and sensors
	private final StatusSignal<Double> driveMotorSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> driveMotorTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> driveDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> driveStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> driveEncoderPostion;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> driveEncoderVelocity;			// Encoder position, in pinion rotations/second

  // Turning motor objects
  private final TalonFX turningMotor;
	private final TalonFXConfigurator turningMotorConfigurator;
	private TalonFXConfiguration turningMotorConfig;
	private VoltageOut turningVoltageControl = new VoltageOut(0.0);
  private PositionVoltage turningPositionControl = new PositionVoltage(0.0);

	// Turning motor signals and sensors
	private final StatusSignal<Double> turningMotorTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> turningDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> turningStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> turningEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> turningEncoderVelocity;			// Encoder Velocity, in pinion rotations/second
	private final StatusSignal<Double> turningCanCoderPosition;			// CanCoder position, in pinion rotations
	private final StatusSignal<Double> turningCanCoderVelocity;			// Encoder Velocity, in pinion rotations/second


  // CANCoder objects
  //private final WPI_CANCoder turningCanCoder;
  private final CANcoder turningCanCoder;
  private final CANcoderConfiguration turningCanCoderConfig;
  private final CANcoderConfigurator turningCanCoderConfigurator;

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

    // Create motor, encoder, signal, and sensor objects
    driveMotor = new TalonFX(driveMotorAddress);
    driveMotorConfigurator = driveMotor.getConfigurator();
    driveMotorSupplyVoltage = driveMotor.getSupplyVoltage();
	  driveMotorTemp = driveMotor.getDeviceTemp();
	  driveDutyCycle = driveMotor.getDutyCycle();
	  driveStatorCurrent = driveMotor.getStatorCurrent();
	  driveEncoderPostion = driveMotor.getPosition();
	  driveEncoderVelocity = driveMotor.getVelocity();

    turningMotor = new TalonFX(turningMotorAddress);
    turningMotorConfigurator = turningMotor.getConfigurator();
	  turningMotorTemp = turningMotor.getDeviceTemp();
	  turningDutyCycle = turningMotor.getDutyCycle();
	  turningStatorCurrent = turningMotor.getStatorCurrent();
	  turningEncoderPosition = turningMotor.getPosition();
	  turningEncoderVelocity = turningMotor.getVelocity();

    //turningCanCoder = new WPI_CANCoder(cancoderAddress);
    turningCanCoder = new CANcoder(cancoderAddress);
    // turningCanCoderConfigurator = turningCanCoderCooler.getConfigurator();
    // turningCanCoderConfig = new CANcoderConfiguration();
    
    // **** Setup drive motor configuration

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
    driveMotorConfig.Slot0.kP = 0.24;		// TODO calibrate.  2023 Phoenix5 = 0.10.  kP = (desired-output-volts) / (error-in-encoder-rps)
		driveMotorConfig.Slot0.kI = 0.0;
		driveMotorConfig.Slot0.kD = 0.000012;    // TODO calibrate.  2023 Phoenix5 = 0.005.  kD = (desired-output-volts) / (error-in-encoder-rps/s)
		// driveMotorConfig.Slot0.kS = 0.0;
		// driveMotorConfig.Slot0.kV = 0.0;
		// driveMotorConfig.Slot0.kA = 0.0;

    // **** Setup turning motor configuration

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
    turningMotorConfig.Slot0.kP = 3.6;		// TODO calibrate.  2023 Phoenix5 = 0.15.  kP = (desired-output-volts) / (error-in-encoder-rotations)
		turningMotorConfig.Slot0.kI = 0.0;
		turningMotorConfig.Slot0.kD = 0.072;    // TODO calibrate.  2023 Phoenix5 = 3.0.  kD = (desired-output-volts) / (error-in-encoder-rps)
		// turningMotorConfig.Slot0.kS = 0.0;
		// turningMotorConfig.Slot0.kV = 0.0;
		// turningMotorConfig.Slot0.kA = 0.0;
    turningCanCoderConfigurator = turningCanCoder.getConfigurator();
    turningCanCoderConfig = new CANcoderConfiguration(); 
    turningCanCoderConfig.MagnetSensor.SensorDirection = cancoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;  //TODO Determine which direction is reversed
    turningCanCoderPosition = turningCanCoder.getPosition();
    turningCanCoderVelocity = turningCanCoder.getVelocity();
    // Configure the swerve module motors and encoders
    configSwerveModule();
    // Error when put in configSwerveModule
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
 		// Apply configuration to the drive motor.
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		driveMotorConfigurator.apply(driveMotorConfig);

 		// Apply configuration to the turning motor.
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		turningMotorConfigurator.apply(turningMotorConfig);

    // Apply configuration to the cancoder
    turningCanCoderConfigurator.apply(turningCanCoderConfig);
    // **** configure turning CanCoder
    // turningCanCoder.configFactoryDefault(100);
    // turningCanCoder.configAllSettings(CTREConfigs.swerveCanCoderConfig, 100);
    // turningCanCoder.configSensorDirection(cancoderReversed, 100);
    // turningCanCoderConfigurator = turningCanCoderCooler.getConfigurator();
    // turningCanCoderConfig = new CANcoderConfiguration();


    

    // NOTE!!! When the Cancoder or TalonFX encoder settings are changed above, then the next call to 
    // getCanCoderDegrees() getTurningEncoderDegrees() may contain an old value, not the value based on 
    // the updated configuration settings above!!!!  The CANBus runs asynchronously from this code, so 
    // sending the updated configuration to the CanCoder/Falcon and then receiving an updated position measurement back
    // may take longer than this code.
    // The timeouts in the configuration code above should take care of this, but it does not always wait long enough.
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
   * Sets the swerve module in coast or brake mode.
   * <p> <b>Note</b> that this procedure includes multiple blocking calls and will delay robot code.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    if (setCoast) {
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      turningMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turningMotor.setNeutralMode(NeutralModeValue.Brake);
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
    setDriveMotorPercentOutput(0.0);
    setTurnMotorPercentOutput(0.0);
  }

  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setDriveMotorPercentOutput(double percentOutput){
    driveMotor.setControl(driveVoltageControl.withOutput(percentOutput*SwerveConstants.voltageCompSaturation));
  }
  
  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setTurnMotorPercentOutput(double percentOutput){
    turningMotor.setControl(turningVoltageControl.withOutput(percentOutput*SwerveConstants.voltageCompSaturation));
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
      setDriveMotorPercentOutput(driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    else {
      driveMotor.setControl(driveVelocityControl
        .withVelocity(calculateDriveEncoderVelocityRaw(desiredState.speedMetersPerSecond))
        .withFeedForward(driveFeedforward.calculate(desiredState.speedMetersPerSecond)*SwerveConstants.voltageCompSaturation));
    }

    // Set turning motor target angle
    // TODO Determine the right way to implement this code.  Make it selectable by a boolean parameter to setDesiredState?
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    // double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeedMetersPerSecond * 0.01)) 
    //   ? getTurningEncoderDegrees() : desiredState.angle.getDegrees(); 
    turningMotor.setControl(turningPositionControl.withPosition(calculateTurningEncoderTargetRaw(desiredState.angle.getDegrees())));
  }

  // ********** Encoder methods

  // ******* Drive encoder methods

  /**
   * @return drive encoder position, in pinion rotations
   */
  public double getDriveEncoderRotations() {
    driveEncoderPostion.refresh();
    return driveEncoderPostion.getValueAsDouble();
  }

  /**
	 * Set the drive encoder position to zero in software.
	 */
  public void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRotations();
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "ZeroDriveEncoder", "driveEncoderZero", driveEncoderZero, "raw encoder", getDriveEncoderRotations(), "encoder meters", getDriveEncoderMeters());
  }

  /**
   * @return drive wheel distance traveled, in meters (+ = forward)
   */
  public double getDriveEncoderMeters() {
    return (getDriveEncoderRotations() - driveEncoderZero) * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * @return drive wheel velocity, in meters per second (+ = forward)
   */
  public double getDriveEncoderVelocity() {
    driveEncoderVelocity.refresh();
    return driveEncoderVelocity.getValueAsDouble() * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * Converts a target velocity (in meters per second) to a target raw drive motor velocity.
   * @param velocityMPS Desired drive wheel velocity, in meters per second (+ = forward)
   * @return drive motor raw value equivalent velocity, in encoder ticks (=pinion rotations) per second
   */
  public double calculateDriveEncoderVelocityRaw(double velocityMPS) {
    return velocityMPS / SwerveConstants.kDriveEncoderMetersPerTick;
  }
  
  // ******* Turning TalonFX encoder methods

  /**
   * @return turning TalonFx encoder position, in pinion rotations
   */
  public double getTurningEncoderRaw() {
    turningEncoderPosition.refresh();
    return turningEncoderPosition.getValueAsDouble();
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
    turningEncoderVelocity.refresh();
    return turningEncoderVelocity.getValueAsDouble() * SwerveConstants.kTurningEncoderDegreesPerTick;
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
    turningCanCoderPosition.refresh();
    // return MathBCR.normalizeAngle(turningCanCoder.getAbsolutePosition() - cancoderZero);
    return MathBCR.normalizeAngle(turningCanCoderPosition.getValueAsDouble() - cancoderZero);
  }

  /**
   * @return turning CanCoder rotational velocity for wheel facing, in degrees per second.
   * + = counterclockwise, - = clockwise
   */
  public double getCanCoderVelocityDPS() {
    turningCanCoderVelocity.refresh();
    return turningCanCoderVelocity.getValueAsDouble();
    //return turningCanCoder.getVelocity();
  }


  // ********** Information methods

  public double getDriveBusVoltage() {
    driveMotorSupplyVoltage.refresh();
    return driveMotorSupplyVoltage.getValueAsDouble();
  }

  public double getDriveOutputPercent() {
    driveDutyCycle.refresh();
    return driveDutyCycle.getValueAsDouble();
  }

  public double getDriveStatorCurrent() {
    driveStatorCurrent.refresh();
    return driveStatorCurrent.getValueAsDouble();
  }

  public double getDriveTemp() {
    driveMotorTemp.refresh();
    return driveMotorTemp.getValueAsDouble();
  }

  public double getTurningOutputPercent() {
    turningDutyCycle.refresh();
    return turningDutyCycle.getValueAsDouble();
  }

  public double getTurningStatorCurrent() {
    turningStatorCurrent.refresh();
    return turningStatorCurrent.getValueAsDouble();
  }

  public double getTurningTemp() {
    turningMotorTemp.refresh();
    return turningMotorTemp.getValueAsDouble();
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
