/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorRegion;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.utilities.ElevatorProfileGenerator;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;

/**
 * Add your docs here.
 */
public class Elevator extends SubsystemBase implements Loggable{
	private final FileLog log;
	private final int logRotationKey;         // key for the logging cycle for this subsystem
	private boolean fastLogging = false;
	private final String subsystemName;
	private final Wrist wrist;				// the wrist subsystem

	private final TalonFX elevatorMotor  = new TalonFX(Ports.CANElevatorMotor);
	private final TalonFXConfigurator elevatorMotorConfigurator = elevatorMotor.getConfigurator();
	private TalonFXConfiguration elevatorMotorConfig;
	private VoltageOut elevatorVoltageControl = new VoltageOut(0.0);
	private MotionMagicVoltage elevatorMMControl = new MotionMagicVoltage(0);
	private NeutralModeValue elevatorNeutralMode = NeutralModeValue.Coast;			// Variable to track the current brake/coast mode for the elevator

	// Variables for motor signals and sensors
	private final StatusSignal<Double> elevatorMotorTemp = elevatorMotor.getDeviceTemp();				// Motor temperature, in degC
	private final StatusSignal<Double> elevatorDutyCycle = elevatorMotor.getDutyCycle();				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> elevatorStatorCurrent = elevatorMotor.getStatorCurrent();		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> elevatorEncoderPostion = elevatorMotor.getPosition();			// Encoder position, in pinion rotations
	private final StatusSignal<Double> elevatorEncoderVelocity = elevatorMotor.getVelocity();			// Encoder position, in pinion rotations/second
	private final StatusSignal<ForwardLimitValue> limitUpperSignal = elevatorMotor.getForwardLimit();	// Forward limit switch
	private final StatusSignal<ReverseLimitValue> limitLowerSignal = elevatorMotor.getReverseLimit();	// Reverse limit switch

	// TODO Delete elevatorProfile
	// private final ElevatorProfileGenerator elevatorProfile;

	private boolean elevCalibrated = false; // true is encoder is working and calibrated, false is not calibrated
	private boolean elevPosControl = false; // true is in position control mode (motion profile), false is manual motor control (percent output)

	
	public Elevator(Wrist wrist, FileLog log) {
		this.log = log;
		logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
		subsystemName = "Elevator";
		this.wrist = wrist;								// Save the wrist subsystem (so elevator can get wrist status)
		wrist.saveElevatorObject(this);					// Pass elevator subsystem to wrist (so wrist can get elevator status)

		// TODO Delete elevatorProfile
		// create elevator motion profile object (do this first, used in "checkAndZeroElevatorEnc")
		// elevatorProfile = new ElevatorProfileGenerator(this, log);	

		// Start with factory default TalonFX configuration
		elevatorMotorConfig = new TalonFXConfiguration();			// Factory default configuration
		// elevatorMotorConfigurator.refresh(elevatorMotorConfig);			// Read current configuration.  This is blocking call, up to the default 50ms.

		// configure motor
		elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;			// Invert motor to make positive move the elevator up
		elevatorMotorConfig.MotorOutput.NeutralMode = elevatorNeutralMode;
		// elevatorMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0;  // Default = 0
		// elevatorMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;			// Default = 1.0.  We probably won't use duty-cycle control, since there is no longer voltage compensation
		// elevatorMotorConfig.MotorOutput.PeakReverseDutyCycle = -1.0;			// Default = -1.0.  We probably won't use duty-cycle control, since there is no longer voltage compensation
		elevatorMotorConfig.Voltage.PeakForwardVoltage = ElevatorConstants.voltageCompSaturation;
		elevatorMotorConfig.Voltage.PeakReverseVoltage = -ElevatorConstants.voltageCompSaturation;
		elevatorMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;		// 0.3 seconds
		// elevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; 		// Calibrate if using Talon PID (currently not being used)

		// Note:  In Phoenix 6, slots are selected in the ControlRequest (ex. VelocityVoltage.Slot)
		elevatorMMControl.Slot = 0;
		elevatorMMControl.OverrideBrakeDurNeutral = true;
		elevatorMotorConfig.Slot0.kP = ElevatorConstants.kP;
		elevatorMotorConfig.Slot0.kI = ElevatorConstants.kI;
		elevatorMotorConfig.Slot0.kD = ElevatorConstants.kD;
		elevatorMotorConfig.Slot0.kS = ElevatorConstants.kS;
		elevatorMotorConfig.Slot0.kV = ElevatorConstants.kV;
		elevatorMotorConfig.Slot0.kA = ElevatorConstants.kA;
		elevatorMotorConfig.Slot0.kG = ElevatorConstants.kG;
		elevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		//set Magic Motion Settings
		elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MMCruiseVelocity;
		elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MMAcceleration;
		elevatorMotorConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.MMJerk;

		// configure encoder on motor
		elevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

		// configure limit switches on motor
		elevatorMotorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
		elevatorMotorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
		elevatorMotorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
		elevatorMotorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
		elevatorMotorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
		elevatorMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		// elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit;
		// elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

		// Apply configuration to the elevator motor.  
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		elevatorMotorConfigurator.apply(elevatorMotorConfig);

		checkAndZeroElevatorEnc();

		// Wait 0.25 seconds before checking the limit switch or encoder ticks.  The reason is that zeroing the encoder (above)
		// or setting the limit switch type (above) can be delayed up to 50ms for a round trip
		// from the Rio to the Talon and back to the Rio.  So, reading position could give the wrong value if
		// we don't wait (random weird behavior).
		// Verified that this is still needed after migrating to Phoenix6.
		// DO NOT GET RID OF THIS WITHOUT TALKING TO DON OR ROB.
		Wait.waitTime(250);

		// start the elevator in manual mode unless it is properly zeroed
		elevCalibrated = (isElevatorAtLowerLimit() && getElevatorEncRotations() == 0);

		// ensure the elevator starts in manual mode
		stopElevator();
	}

	/**
	 * Returns the name of the subsystem
	 */
	public String getName() {
		return subsystemName;
	}

	/**
	 * Sets the brake/coast mode for the elevator motor.  
	 * <p> Note that this method is blocking, so it may delay 
	 * robot code (use sparingly).  If the motor is already
	 * in the requested brake/coast mode, then this method
	 * will not do anything and will not block.
	 * @param setCoast true = coast mode, false = brake mode
	 */
	public void setMotorModeCoast(boolean setCoast) {
		if (setCoast) {
			if (elevatorNeutralMode==NeutralModeValue.Coast) return;
			elevatorNeutralMode = NeutralModeValue.Coast;
		} else {
			if (elevatorNeutralMode==NeutralModeValue.Brake) return;
			elevatorNeutralMode = NeutralModeValue.Brake;
		}
		// elevatorMotorConfig.MotorOutput.NeutralMode = elevatorNeutralMode;
		// elevatorMotorConfigurator.apply(elevatorMotorConfig.MotorOutput);
		elevatorMotor.setNeutralMode(elevatorNeutralMode);
	}
	
	// ************ Elevator movement methods

	/**
	 * stops elevator motors
	 */
	public void stopElevator() {
		setElevatorMotorPercentOutput(0.0);
	}

	/**
	 * Sets elevator to the specified percent output of the voltage compensation limit.
	 * <p><b>This method is for internal use only.</b>  It does not change manual vs profile
	 * control modes or check interlocks.
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	private void setElevatorMotorPercentOutputDirect(double percentOutput) {
		elevatorMotor.setControl(elevatorVoltageControl.withOutput(percentOutput*ElevatorConstants.voltageCompSaturation));
	}

	/**
	 * Sets elevator to manual control mode with the specified percent output of the voltage compensation limit.
	 * Does not move the elevator up if the wrist is in the back region (interlock to prevent crashing).
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevPosControl = false;

		// TODO delete elevatorProfile
		// elevatorProfile.disableProfileControl();

		// Clamp speed depending on calibration
		if (elevCalibrated) {
			percentOutput = MathUtil.clamp(percentOutput, 
			-ElevatorConstants.maxManualPercentOutput, ElevatorConstants.maxManualPercentOutput);
		} else {
			percentOutput = MathUtil.clamp(percentOutput, 
			-ElevatorConstants.maxUncalibratedPercentOutput, ElevatorConstants.maxUncalibratedPercentOutput);
		}

		// Do not move the elevator up if the wrist is in the back region (interlock to prevent crashing).
		if (wrist.getWristRegion() == WristRegion.back && percentOutput > 0.0) {
			percentOutput = 0.0;
		}

		setElevatorMotorPercentOutputDirect(percentOutput);
	}

	/**
	 * Gets the elevator percent output of the voltage compensation limit.
	 * @return between -1.0 (down) and 1.0 (up)
	 */
	public double getElevatorMotorPercentOutput() {
		elevatorDutyCycle.refresh();			// Verified that this is not a blocking call.
		return elevatorDutyCycle.getValueAsDouble();
	}

	/**
	 * Sets target position for elevator, using motion profile movement.
	 * This only works when encoder is working and elevator is calibrated and the wrist is not interlocked.
	 * @param pos in inches, per ElevatorConstants.ElevatorPosition
	*/
	public void setProfileTarget(double pos) {
		if (elevCalibrated) {
			elevPosControl = true;

			pos = MathUtil.clamp(pos, ElevatorPosition.lowerLimit.value, ElevatorPosition.upperLimit.value);

			// Do not move the elevator out of the bottom region if the wrist is not in the main region (interlock to prevent crashing).
			if (wrist.getWristRegion() != WristRegion.main && pos > ElevatorConstants.boundBottomMain) {
				pos = ElevatorConstants.boundBottomMain;
			}

			// TODO delete elevatorProfile
			// elevatorProfile.setProfileTarget(pos);
			elevatorMotor.setControl(elevatorMMControl.withPosition(pos/ElevatorConstants.kElevEncoderInchesPerTick));			// Calculated desired position in encoder rotations

			SmartDashboard.putNumber("ElevatorInitPos", getElevatorPos());
			SmartDashboard.putNumber("ElevatorTarget", pos);
	
			log.writeLog(false, subsystemName, "setProfileTarget", "Cur pos", getElevatorPos(), "Target", pos, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, subsystemName, "setProfileTarget", "Cur pos", getElevatorPos(), "Target", pos, "Allowed,No,Wrist Angle",
 			  wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		}
	}

	/**
	 * Returns the height that elevator is trying to move to, in inches relative to ElevatorConstants.ElevatorPosition.
	 * <p><b>NOTE:</b> This is the target height, not the current height.
	 * If the elevator is in manual control mode, returns the actual elevator position.
	 * If the elevator is not calibrated, then returns +10in into the main region (where wrist interlock is engaged).
	 * @return desired inches of elevator height
	 */
	public double getCurrentElevatorTarget() {
		if (elevCalibrated) {
			if (elevPosControl) {
				// Motion profile control
				return elevatorMMControl.Position*ElevatorConstants.kElevEncoderInchesPerTick;

				// TODO delete elevatorProfile
				// return elevatorProfile.getFinalPosition();
			} else {
				// Manual control mode
				return getElevatorPos();
			}
		} else {
			return ElevatorConstants.boundBottomMain + 10.0;
		}
	}

	// ************ Encoder methods

	/**
	 * Checks if the elevator is at the lower limit switch.  If it is, then
	 * zeros the elevator encoder.
	 * <p> NOTE:  If the encoder is zeroed, then this method waits up to 100ms
	 * for the new setting to be applied.
	 */
	public void checkAndZeroElevatorEnc() {
		if (isElevatorAtLowerLimit()) {
			stopElevator();			// Make sure Talon PID loop or motion profile won't move the robot to the last set position when we reset the enocder position

			elevatorMotor.setPosition(0, 0.100);			// Verified that this is a blocking call (typ ~6ms), but the next read may still not see the zeroed position.
			elevCalibrated = true;

			log.writeLog(true, subsystemName, "Calibrate and Zero Encoder", "checkAndZeroElevatorEnc");
		}
	}

	/**
	 * Returns if the encoder is calibrated and working
	 * @return true = working, false = not working
	 */
	public boolean encoderCalibrated() {
		return elevCalibrated;
	}

	/**
	 * @return raw encoder reading, in pinion rotations (based on encoder zero being at zero position)
	 */
	public double getElevatorEncRotations() {
		elevatorEncoderPostion.refresh();			// Verified that this is not a blocking call.
		return elevatorEncoderPostion.getValueAsDouble();
	}

	/**
	 * @return Current elevator position in inches, per ElevatorConstants.ElevatorPosition 
	 * If the elevator is not calibrated, then returns +10in into the main region (where wrist interlock is engaged).
	 * 
	 */
	public double getElevatorPos() {
		if (elevCalibrated) {
			return getElevatorEncRotations()*ElevatorConstants.kElevEncoderInchesPerTick;
		} else {
			return ElevatorConstants.boundBottomMain + 10.0;
		}
	}

	/**
	 * Returns the elevator region for a given position, relative to ElevatorConstants.ElevatorPosition (in inches).
	 * <p> This is a private method, only for use in this subsystem!!!!
	 * @param pos in inches, per ElevatorConstants.ElevatorPosition
	 * @return current elevatorRegion
	 */
	private ElevatorRegion calcElevatorRegion(double pos) {
		if (elevCalibrated) {
			if (pos >= ElevatorConstants.boundBottomMain) {
				return ElevatorRegion.main;
			 } else {
				return ElevatorRegion.bottom;
			 }
		} else {
			return ElevatorRegion.uncalibrated;
		}
	}

	/**
	 * Returns the elevator region that the elevator is currently in.  If the elevator is moving up, value will return "main".
	 * @return current elevatorRegion
	 */
	public ElevatorRegion getElevatorRegion() {
		if (elevCalibrated) {
			if (getElevatorMotorPercentOutput() > 0.05 ||
				(elevPosControl && calcElevatorRegion(getCurrentElevatorTarget())==ElevatorRegion.main) 
			   ) {
				return ElevatorRegion.main;		// Elevator is moving up.  For safety, assume we are in the main region
			} else {
				return calcElevatorRegion(getElevatorPos());
			}
		} else {
			return ElevatorRegion.uncalibrated;
		}
	}

	/**
	 * @return Current elevator velocity in in/s, + equals up, - equals down
	 */
	public double getElevatorVelocity() {
		elevatorEncoderVelocity.refresh();			// Verified that this is not a blocking call.
		return elevatorEncoderVelocity.getValueAsDouble() * ElevatorConstants.kElevEncoderInchesPerTick;
	}

	// ************ Sensor methods

	/**
	 * reads whether the elevator is at the upper limit
	 */
	public boolean isElevatorAtUpperLimit() {
		limitUpperSignal.refresh();			// Verified that this is not a blocking call.
		return limitUpperSignal.getValue() == ForwardLimitValue.ClosedToGround;
	}

	/**
	 * reads whether the elevator is at the lower limit
	 */
	public boolean isElevatorAtLowerLimit() {
		limitLowerSignal.refresh();			// Verified that this is not a blocking call.
		return limitLowerSignal.getValue() == ReverseLimitValue.ClosedToGround;
	}

	// ************ Periodic and information methods

	/**
    * Writes information about the subsystem to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
	public void updateElevatorLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
				"Temp", elevatorMotorTemp.refresh().getValueAsDouble(),
				"PctOut", getElevatorMotorPercentOutput(), "Amps", elevatorStatorCurrent.refresh().getValueAsDouble(),
				"Enc Rots", getElevatorEncRotations(), "Enc Inches", getElevatorPos(), 
				"Elev Target", getCurrentElevatorTarget(), "Elev Vel", getElevatorVelocity(),
				"Upper Limit", isElevatorAtUpperLimit(), "Lower Limit", isElevatorAtLowerLimit(),
				"Elev Calibrated", elevCalibrated, "Elev Pos Control", elevPosControl);
	}

	/**
	 * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
	 * @param enabled true = every cycle, false = every 10 cycles
	 */ 
	@Override
	public void enableFastLogging(boolean enabled) {
		fastLogging = enabled;
	}
	
	@Override
	public void periodic() {
		
		if (log.isMyLogRotation(logRotationKey)) {
			SmartDashboard.putBoolean("Elev Calibrated", elevCalibrated);
			SmartDashboard.putBoolean("Elev Pos Control", elevPosControl);
			SmartDashboard.putNumber("Elev Pos", getElevatorPos());
			SmartDashboard.putNumber("Elev Target", getCurrentElevatorTarget());
			SmartDashboard.putNumber("Elev Ticks", getElevatorEncRotations());
			SmartDashboard.putBoolean("Elev Lower Limit", isElevatorAtLowerLimit());
			SmartDashboard.putBoolean("Elev Upper Limit", isElevatorAtUpperLimit());
		}

		if (fastLogging || log.isMyLogRotation(logRotationKey)) {
			updateElevatorLog(false);
		}

		// Sets elevator motors to percent power required as determined by motion profile.
		// Only set percent power IF the motion profile is enabled.
		// Note:  If we are using our motion profile control loop, then set the power directly using setElevatorMotorPercentOutputDirect().
		// Do not call setElevatorMotorPercentOutput(), since that will change the elevPosControl to false (manual control).
		if (elevPosControl) {
			// TODO delete elevatorProfile
			// setElevatorMotorPercentOutputDirect(elevatorProfile.trackProfilePeriodic());

			// TODO if elevator is moving (not within tolerance of target position, then log similar to this)
			// log.writeLog(logWhenDisabled, "ElevatorProfile", "updateCalc",
			// 	"MP Pos", getCurrentPosition(), "ActualPos", elevator.getElevatorPos(), 
			// 	"TargetPos", finalPosition, "Time since start", getTimeSinceProfileStart(), "dt", dt,
			// 	"ActualVel", elevator.getElevatorVelocity(),
			// 	"MP Vel", (currentMPVelocity * directionSign),
			// 	"MP Accel", (currentMPAcceleration * directionSign),
			// 	"PowerFF", percentPowerFF, "PowerFB", percentPowerFB );
		}

		// If in manual drive mode, 
   		// then enforce interlocks (stop elevator if at edge of allowed region based on wrist)
		if (!elevPosControl && elevCalibrated) {
			// Do not move the elevator up if the wrist is in the back region (interlock to prevent crashing).
			if (wrist.getWristRegion() == WristRegion.back && getElevatorMotorPercentOutput() > 0.0) {
				stopElevator();
			}
		}

		// Autocalibrate if the encoder is OK and the elevator is at the lower limit switch
		if (!elevCalibrated || Math.abs(getElevatorEncRotations()) > 2.5) {
			checkAndZeroElevatorEnc();
		}
	}
}