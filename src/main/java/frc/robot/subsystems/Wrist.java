/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CTREConfigs;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ElevatorConstants.ElevatorRegion;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase implements Loggable{
  private final FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false;
  private final String subsystemName;
  private Elevator elevator;        // Do not call this elevator object in the Wrist constructor!  The elevator constructor will set this variable (after the wrist constructor).

  private final TalonFX wristMotor = new TalonFX(Ports.CANWristMotor);
  
  private final DutyCycleEncoder revEncoder = new DutyCycleEncoder(Ports.DIOWristRevThroughBoreEncoder);

  private double revEncoderZero = 0;          // Reference raw encoder reading for encoder.  Calibration sets this to the absolute position from RobotPreferences.
  private double wristCalZero = 0;   		      // Wrist encoder position at O degrees, in degrees (i.e. the calibration factor).  Calibration sets this to match the REV through bore encoder.
  private boolean wristCalibrated = false;    // Default to wrist being uncalibrated.  Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard

  private double safeAngle;         // current wrist target on position control on the Falcon motor (if the Falcon is in position mode)

  public Wrist(FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
    subsystemName = "Wrist";

    // Configure motor
    wristMotor.configFactoryDefault(100);
    wristMotor.configAllSettings(CTREConfigs.wristFXConfig, 100);
    wristMotor.selectProfileSlot(0, 0);
    wristMotor.setInverted(true);           // Motor needs to be inverted due to config on robot
    wristMotor.enableVoltageCompensation(true);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    stopWrist();

    // Configure encoder on motor
		wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		wristMotor.setSensorPhase(false);         // True = Flip direction of sensor reading
    wristMotor.configFeedbackNotContinuous(false, 100);

    // Rev Through-Bore Encoder settings
    // Wait for through-bore encoder to connect, up to 0.25 sec
    long t = System.currentTimeMillis() + 250;
    while (System.currentTimeMillis() < t && !isRevEncoderConnected());    
    if (isRevEncoderConnected()) {
      // Copy calibration to wrist encoder
      calibrateRevEncoderDegrees(revEncoderOffsetAngleWrist);
      wristCalibrated = true;
    } else {
      wristCalibrated = false;
      RobotPreferences.recordStickyFaults("Wrist-ThroughBoreEncoder", log);
    }

    // Wait 0.25 seconds before adjusting the wrist calibration.  The reason is that .setInverted (above)
    // changes the sign of read encoder value, but that change can be delayed up to 50ms for a round trip
    // from the Rio to the Talon and back to the Rio.  So, reading angles could give the wrong value if
    // we don't wait (random weird behavior).
    // DO NOT GET RID OF THIS WITHOUT TALKING TO DON OR ROB.
    Wait.waitTime(250);

    if (wristCalibrated) {
      calibrateWristEnc(getRevEncoderDegrees());

      // adjustWristCalZero();

      // Configure soft limits on motor
      wristMotor.configForwardSoftLimitThreshold(wristDegreesToEncoderTickPosition(WristAngle.upperLimit.value), 100);
      wristMotor.configReverseSoftLimitThreshold(wristDegreesToEncoderTickPosition(WristAngle.lowerLimit.value), 100);
      wristMotor.configForwardSoftLimitEnable(true, 100);
      wristMotor.configReverseSoftLimitEnable(true, 100);
    }
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Saves a copy of the elevator object for future use.
   * <p> The Elevator constructor <b>must</b> call this method.
   * @param elevator elevator subsystem
   */
  public void saveElevatorObject(Elevator elevator) {
    this.elevator = elevator;
  }

	// ************ Wrist movement methods

  /**
   * Stops wrist motor
   */
  public void stopWrist() {
    setWristMotorPercentOutput(0.0);
  }

  /**
   * Sets percent power of wrist motor
   * <p><b> There are no elevator interlocks on this method!!!! </b>
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setWristMotorPercentOutput(double percentOutput) {
    if (wristCalibrated) {
      percentOutput = MathUtil.clamp(percentOutput, -maxPercentOutput, maxPercentOutput);
    } else {
      percentOutput = MathUtil.clamp(percentOutput, -maxUncalibratedPercentOutput, maxUncalibratedPercentOutput);
    }

    wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Only works when encoder is working and calibrated
   * Interlocks with elevator position
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle) {
    if (wristCalibrated) {
      // Keep wrist in usable range
      safeAngle = MathUtil.clamp(angle, WristAngle.lowerLimit.value, WristAngle.upperLimit.value);

      WristRegion curRegion = getRegion(getWristAngle());

      // Check elevator interlocks
      if (curRegion == WristRegion.main && elevator != null ) {
        if (elevator.getElevatorRegion() == ElevatorRegion.main) {
          safeAngle = MathUtil.clamp(safeAngle, boundBackMain, WristAngle.upperLimit.value);
        }
      }

      wristMotor.set(ControlMode.Position, wristDegreesToEncoderTickPosition(safeAngle), 
        DemandType.ArbitraryFeedForward, kG * Math.cos(safeAngle*Math.PI/180.0));

      if (elevator != null) {
        log.writeLog(false, subsystemName, "Set angle", "Desired angle", angle, "Set angle", safeAngle,
        "Elevator Pos", elevator.getElevatorPos(), "Elevator Target", elevator.getCurrentElevatorTarget());
      } else {
        log.writeLog(false, subsystemName, "Set angle", "Desired angle", angle, "Set angle", safeAngle,
        "Elevator", "null object");
      }
      SmartDashboard.putNumber("Wrist set raw ticks", wristDegreesToEncoderTickPosition(safeAngle));
    }
  }

  /**
	 * Returns the angle that wrist is trying to move to in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit in backFar region to engage all interlocks,
   * since we really don't know where the wrist is at.  If the wrist is in manual control mode, then
   * returns the actual wrist position.
	 * @return desired degree of wrist angle
	 */
  public double getCurrentWristTarget() {
    double currentTarget;

    if (wristCalibrated) {
      if (wristMotor.getControlMode() == ControlMode.Position) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we aren't moving towards a target (and the target
        // angle may be undefined).  So, get the actual wrist angle instead.
        currentTarget = getWristAngle();
      }
      return currentTarget;
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

	// ************ Wrist region methods

 	/**
   * For use in the wrist subsystem only.  Use getWristRegion() when calling from outside this class.
	 * <p>Returns the wrist region for a given angle.
   * @param degrees angle in degrees
	 * @return corresponding wrist region
	 */
	private WristRegion getRegion(double degrees) {
      if (degrees <= boundBackMain) return WristRegion.back;
      else return WristRegion.main; 
	}

	/**
	 * Returns the wrist region that the wrist is currently in.  If the wrist is moving between regions, the
   * value will return the more restrictive of the two regions.
	 * @return current wristRegion
	 */
	public WristRegion getWristRegion() {
    if (!wristCalibrated) {
      return WristRegion.uncalibrated;
    }

    WristRegion curRegion = getRegion(getWristAngle());

    if (wristMotor.getControlMode() == ControlMode.Position) {
      WristRegion targetRegion = getRegion(safeAngle);

      if (targetRegion != WristRegion.main) curRegion = WristRegion.back;
    }
      
    return curRegion;
	}

	// ************ Internal Falcon encoder methods

  /**
   * 
   * @return raw encoder ticks, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoderTicksRaw() {
    return wristMotor.getSelectedSensorPosition(0);
  }

  /**
   * Converts the wrist position in degrees to raw encoder ticks.  Assumes that the encoder is calibrated.
   * @param degrees wrist position in degrees
   * @return wrist position in encoder ticks
   */
  private double wristDegreesToEncoderTickPosition(double degrees) {
    return (degrees + wristCalZero) / kWristDegreesPerTick;
  }

  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * Assumes that the encoder is calibrated.
   * <p>Gets the current wrist angle from the Falcon encoder.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    // DO NOT normalize this angle.  It should not wrap, since the wrist mechanically can not cross the -180/+180 deg point
    return getWristEncoderTicksRaw()* kWristDegreesPerTick - wristCalZero;
  }

  /**
	 * Returns the angle that wrist is currently positioned at in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit in backFar region to engage all interlocks,
   * since we really don't know where the wrist is at.
	 * @return current degree of wrist angle
	 */
  public double getWristAngle() {
    if (wristCalibrated) {
      return getWristEncoderDegrees();
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

  // ************ Internal Falcon encoder calibration methods

  /**
	 * returns whether encoder is calibrated or not
	 * @return true if encoder is calibrated and working, false if encoder broke
	 */
	public boolean isEncoderCalibrated() {
		return wristCalibrated;
  }

	/**
	 * Stops wrist motor and sets wristCalibrated to false
	 */
	public void setWristUncalibrated() {
		stopWrist();

    log.writeLog(false, "Wrist", "Uncalibrate wrist", 
      "Rev angle", getRevEncoderDegrees(), "Enc Raw", getWristEncoderTicksRaw(),
			"Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());

    wristCalibrated = false;
  }

  /**
   * Calibrates the wrist encoder, assuming we know the wrist's current angle
   * @param angle current angle that the wrist is physically at, in degrees
   */
  public void calibrateWristEnc(double angle) {
		stopWrist();	// Stop motor, so it doesn't jump to new value

    wristCalZero = getWristEncoderTicksRaw()* kWristDegreesPerTick - angle;
		wristCalibrated = true;

    log.writeLog(false, "Wrist", "Calibrate wrist", "zero value", wristCalZero, 
			"Rev angle", getRevEncoderDegrees(), "Enc Raw", getWristEncoderTicksRaw(),
			"Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());
  }  
  
	/**
	 * If the angle is reading >/< max/min angle, add/subtract 360 degrees to the wristCalZero accordingly
	 * Note: when the motor is not inverted, upon booting up, an absolute encoder reads a value between 0 and 2048
	 * 		 when the motor is inverted, upon booting up, an absolute encoder reads a value between 0 and -2048????  (that was true for mag-encoder.  is it true for Falcon internal encoder?)
	 * Note: absolute encoder values don't wrap during operation
	 */
	// public void adjustWristCalZero() {
  //   log.writeLogEcho(false, subsystemName, "Adjust wrist pre", "wrist angle", getWristAngle(), 
  //     "raw ticks", getWristEncoderTicksRaw(), "wristCalZero", wristCalZero);
	// 	if(getWristAngle() < WristAngle.lowerLimit.value - 15.0) {
  //     log.writeLogEcho(false, subsystemName, "Adjust wrist", "Below min angle");
	// 		wristCalZero -= WristConstants.kEncoderCPR;
	// 	}
	// 	else if(getWristAngle() > WristAngle.upperLimit.value + 10.0) {
  //     log.writeLogEcho(false, subsystemName, "Adjust wrist", "Above max angle");
	// 		wristCalZero += WristConstants.kEncoderCPR;
	// 	}
  //   log.writeLogEcho(false, subsystemName, "Adjust wrist post", "wrist angle", getWristAngle(), 
  //     "raw ticks", getWristEncoderTicksRaw(), "wristCalZero", wristCalZero);
	// }

 	// ************ REV through-bore encoder methods

  /**
   * Calibrates the REV through bore encoder, so that 0 should be with the CG of the wrist horizontal 
   * facing away from the robot, and -90 deg is with the CG of the wrist resting downward.
   * @param offsetDegrees Desired encoder zero angle, in absolute magnet position reading
   */
  public void calibrateRevEncoderDegrees(double offsetDegrees) {
    revEncoderZero = -offsetDegrees;
    log.writeLogEcho(true, subsystemName, "calibrateThroughBoreEncoder", "encoderZero", revEncoderZero, 
        "raw encoder", revEncoder.get()*360.0, "encoder degrees", getRevEncoderDegrees());
  }

  /**
   * Returns status of the Rev through bore encoder
   * @return true = encoder is connected, false = encoder not connected
   */
  public boolean isRevEncoderConnected() {
    return revEncoder.isConnected();
  }

  /**
   * @return Wrist orientation measured by the REV through bore encoder wrist facing, in degrees [-180,+180).
   * When calibrated, 0 should be with the CG of the wrist horizontal 
   * facing away from the robot, and -90 deg is with the CG of the wrist resting downward.
   */
  public double getRevEncoderDegrees() {
    return MathBCR.normalizeAngle(revEncoder.get()*360.0 - revEncoderZero);
  }

 	// ************ Periodic and information methods

  /**
   * Writes information about the subsystem to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateWristLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Temp", wristMotor.getTemperature(), "Percent Output", wristMotor.getMotorOutputPercent(),
      "Amps", wristMotor.getStatorCurrent(),
      "WristCalZero", wristCalZero, "Enc Raw", getWristEncoderTicksRaw(), 
      "Wrist Degrees", getWristEncoderDegrees(),
      "Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget(),
      "Rev Connected", isRevEncoderConnected(), "Rev Degrees", getRevEncoderDegrees()
    );
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
      SmartDashboard.putBoolean("Wrist Rev connected", isRevEncoderConnected());
      SmartDashboard.putBoolean("Wrist calibrated", wristCalibrated);
      SmartDashboard.putNumber("Wrist Rev angle", getRevEncoderDegrees());
      SmartDashboard.putNumber("Wrist angle", getWristAngle());
      SmartDashboard.putNumber("Wrist enc raw", getWristEncoderTicksRaw());
      SmartDashboard.putNumber("Wrist target angle", getCurrentWristTarget());
      SmartDashboard.putNumber("Wrist output", wristMotor.getMotorOutputPercent());
    }
        
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateWristLog(false);
    }

    // Un-calibrates the wrist if the angle is outside of bounds.
    // Turned off for right now.  It still occasionally false-triggers, even with 20degree tolerances.
    // if (getWristAngle() > WristAngle.upperLimit.value + 20.0 || getWristAngle() < WristAngle.lowerLimit.value - 20.0) {
    //   setWristUncalibrated();
    //   updateWristLog(true);
    // }

    // If in manual drive mode and if elevator object exists, 
    // then enforce interlocks (stop wrist if at edge of allowed region based on elevator)
    if (wristMotor.getControlMode() == ControlMode.PercentOutput && elevator != null && wristCalibrated) {
      double angle = getWristEncoderDegrees();
      double pct = wristMotor.getMotorOutputPercent();
      double tol = 1.0;     // degrees tolerance for safeties

      switch (elevator.getElevatorRegion()) {
        case uncalibrated:
          // No interlock.  Danger zone!!!!!!!
          if ( (angle <= (WristAngle.lowerLimit.value+tol) && pct < 0.0) ||
               (angle >= (WristAngle.upperLimit.value-tol) && pct > 0.0)
             ) {
            stopWrist();
          }
          break;
        case bottom:
          if ( (angle <= (WristAngle.lowerLimit.value+tol) && pct < 0.0) ||
               (angle >= (WristAngle.upperLimit.value-tol) && pct > 0.0)
             ) {
            stopWrist();
          }
          break;
        case main:
          if ( (angle <= (boundBackMain+tol) && pct < 0.0) ||
              (angle >= (WristAngle.upperLimit.value-tol) && pct > 0.0)
              ) {
            stopWrist();
          }
          break;
      }
    }
  }
 
}
