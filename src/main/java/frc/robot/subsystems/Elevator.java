/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CTREConfigs;
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

	private final WPI_TalonFX elevatorMotor  = new WPI_TalonFX(Ports.CANElevatorMotor);
	private final TalonFXSensorCollection elevatorLimits;

	private final ElevatorProfileGenerator elevatorProfile;

	private boolean elevCalibrated = false; // true is encoder is working and calibrated, false is not calibrated
	private boolean elevPosControl = false; // true is in position control mode (motion profile), false is manual motor control (percent output)

	
	public Elevator(Wrist wrist, FileLog log) {
		this.log = log;
		logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
		subsystemName = "Elevator";
		this.wrist = wrist;								// Save the wrist subsystem (so elevator can get wrist status)
		wrist.saveElevatorObject(this);					// Pass elevator subsystem to wrist (so wrist can get elevator status)

		// create elevator motion profile object (do this first, used in "checkAndZeroElevatorEnc")
		elevatorProfile = new ElevatorProfileGenerator(this, log);	

		// configure motor
		elevatorMotor.configFactoryDefault(100);
		elevatorMotor.configAllSettings(CTREConfigs.elevatorFXConfig, 100);
		elevatorMotor.selectProfileSlot(0, 0);
		elevatorMotor.setInverted(true);
		elevatorMotor.enableVoltageCompensation(true);
		setMotorModeCoast(true);

		// configure encoder on motor
		elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		elevatorMotor.setSensorPhase(false);         // True = Flip direction of sensor reading

		// configure limit switches on motor
		elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 100);
		elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 100);
		elevatorMotor.overrideLimitSwitchesEnable(true);
		// elevatorMotor.configForwardSoftLimitThreshold(limit, 100);
		// elevatorMotor.configForwardSoftLimitEnable(true, 100);

		elevatorLimits = elevatorMotor.getSensorCollection();
		checkAndZeroElevatorEnc();

		// Wait 0.25 seconds before checking the limit switch or encoder ticks.  The reason is that zeroing the encoder (above)
		// or setting the limit switch type (above) can be delayed up to 50ms for a round trip
		// from the Rio to the Talon and back to the Rio.  So, reading position could give the wrong value if
		// we don't wait (random weird behavior).
		// DO NOT GET RID OF THIS WITHOUT TALKING TO DON OR ROB.
		Wait.waitTime(250);

		// start the elevator in manual mode unless it is properly zeroed
		elevCalibrated = (isElevatorAtLowerLimit() && getElevatorEncTicks() == 0);

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
	 * @param setCoast true = coast mode, false = brake mode
	 */
	public void setMotorModeCoast(boolean setCoast) {
		if (setCoast) {
			elevatorMotor.setNeutralMode(NeutralMode.Coast);
		} else {
			elevatorMotor.setNeutralMode(NeutralMode.Brake);
		}
	}
	
	// ************ Elevator movement methods

	/**
	 * stops elevator motors
	 */
	public void stopElevator() {
		setElevatorMotorPercentOutput(0.0);
	}

	/**
	 * Sets elevator to manual control mode with the specified percent output voltage.
	 * Does not move the elevator up if the wrist is in the back region (interlock to prevent crashing).
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevPosControl = false;
		elevatorProfile.disableProfileControl();

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

		elevatorMotor.set(ControlMode.PercentOutput, percentOutput);
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

			elevatorProfile.setProfileTarget(pos);

			log.writeLog(false, subsystemName, "setProfileTarget", "Target", pos, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, subsystemName, "setProfileTarget", "Target", pos, "Allowed,No,Wrist Angle",
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
				return elevatorProfile.getFinalPosition();
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
	 * only zeros elevator encoder when it is at the zero position (lower limit)
	 */
	public void checkAndZeroElevatorEnc() {
		if (isElevatorAtLowerLimit()) {
			stopElevator();			// Make sure Talon PID loop or motion profile won't move the robot to the last set position when we reset the enocder position
			elevatorMotor.setSelectedSensorPosition(ElevatorPosition.lowerLimit.value, 0, 100);
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
	 * @return raw encoder ticks (based on encoder zero being at zero position)
	 */
	public double getElevatorEncTicks() {
		return elevatorMotor.getSelectedSensorPosition(0);
	}

	/**
	 * @return Current elevator position, per ElevatorConstants.ElevatorPosition 
	 * If the elevator is not calibrated, then returns +10in into the main region (where wrist interlock is engaged).
	 * 
	 */
	public double getElevatorPos() {
		if (elevCalibrated) {
			return getElevatorEncTicks()*ElevatorConstants.kElevEncoderInchesPerTick;
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
			if (elevatorMotor.getMotorOutputPercent() > 0.05 ||
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
		return elevatorMotor.getSelectedSensorVelocity(0) * ElevatorConstants.kElevEncoderInchesPerTick * 10.0;
	}

	// ************ Sensor methods

	/**
	 * reads whether the elevator is at the upper limit
	 */
	public boolean isElevatorAtUpperLimit() {
		return elevatorLimits.isFwdLimitSwitchClosed() == 1;
	}

	/**
	 * reads whether the elevator is at the lower limit
	 */
	public boolean isElevatorAtLowerLimit() {
		return elevatorLimits.isRevLimitSwitchClosed() == 1;
	}

	// ************ Periodic and information methods

	/**
    * Writes information about the subsystem to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
	public void updateElevatorLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
				"Temp", elevatorMotor.getTemperature(),
				"PctOut", elevatorMotor.getMotorOutputPercent(), "Amps", elevatorMotor.getStatorCurrent(),
				"Enc Ticks", getElevatorEncTicks(), "Enc Inches", getElevatorPos(), 
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
			SmartDashboard.putNumber("Elev Ticks", getElevatorEncTicks());
			SmartDashboard.putBoolean("Elev Lower Limit", isElevatorAtLowerLimit());
			SmartDashboard.putBoolean("Elev Upper Limit", isElevatorAtUpperLimit());
		}

		if (fastLogging || log.isMyLogRotation(logRotationKey)) {
			updateElevatorLog(false);
		}

		// Sets elevator motors to percent power required as determined by motion profile.
		// Only set percent power IF the motion profile is enabled.
		// Note:  If we are using our motion profile control loop, then set the power directly using elevatorMotor.set().
		// Do not call setElevatorMotorPercentOutput(), since that will change the elevPosControl to false (manual control).
		if (elevPosControl) {
			elevatorMotor.set(ControlMode.PercentOutput, elevatorProfile.trackProfilePeriodic());  
		}

		// If in manual drive mode, 
   		// then enforce interlocks (stop elevator if at edge of allowed region based on wrist)
		if (!elevPosControl && elevCalibrated) {
			// Do not move the elevator up if the wrist is in the back region (interlock to prevent crashing).
			if (wrist.getWristRegion() == WristRegion.back && elevatorMotor.getMotorOutputPercent() > 0.0) {
				stopElevator();
			}
		}

		// Autocalibrate if the encoder is OK and the elevator is at the lower limit switch
		if (!elevCalibrated || Math.abs(getElevatorEncTicks()) > 5000) {
			checkAndZeroElevatorEnc();
		}
	}
}