// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

public class Intake extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; 

  private final String subsystemName;

  private final WPI_TalonSRX motor1;
  private final WPI_TalonSRX motor2;
  private final DoubleSolenoid pneumaticDoubleSolenoidLeft;
  private final DoubleSolenoid pneumaticDoubleSolenoidRight;

  private boolean pistonExtended = false;     // Default state = retracted

  /**
   * Constructs the Intake subsystem, including rollers and a solenoid to change between deploy and stowed.
   * @param log object for logging
   */
  public Intake(FileLog log) {
    motor1 = new WPI_TalonSRX(Ports.CANIntake1);
    motor2 = new WPI_TalonSRX(Ports.CANIntake2);
    pneumaticDoubleSolenoidLeft = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolIntakeLeftFwd, Ports.SolIntakeLeftRev);
    pneumaticDoubleSolenoidRight = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolIntakeRightFwd, Ports.SolIntakeRightRev);
    subsystemName = "Intake";
    this.log = log;
    
    logRotationKey = log.allocateLogRotation();

    motor1.setNeutralMode(NeutralMode.Coast);
    motor1.setInverted(true);
    motor1.configVoltageCompSaturation(12.0, 100);
    motor1.enableVoltageCompensation(true);
    motor1.configOpenloopRamp(0.3, 100);     //seconds from neutral to full

    motor2.setNeutralMode(NeutralMode.Coast);
    motor2.setInverted(true);
    motor2.configVoltageCompSaturation(12.0, 100);
    motor2.enableVoltageCompensation(true);
    motor2.configOpenloopRamp(0.3, 100);     //seconds from neutral to full
  }

    /**
   * Returns the name of the subsystem
   */
  public String getName(){
    return subsystemName;
  }

  /**
   * Sets the percent of the motor, + is intake, - is outtake
   * @param percent1 -1.0 to +1.0 percent for motor 1, motor that deploys
   * @param percent2 -1.0 to +1.0 percent for motor 2, motor that doesn't deploy
   */
  public void setMotorPercentOutput(double percent1, double percent2){
    motor1.set(percent1);
    motor2.set(percent2);
  }

  /**
   * Stops the motor
   */
  public void stopMotor(){
    motor1.stopMotor();
    motor2.stopMotor();
  }

  /**
   * @return stator current of motor1 in amps(Motor that deploys)
   */
  public double getMotor1Amps(){
    return motor1.getStatorCurrent();
  }

  /**
   * @return stator current of motor2 in amps(Motor that doesn't deploy)
   */
  public double getMotor2Amps(){
    return motor2.getStatorCurrent();
  }

  /**
   * Sets if the piston should be extended or not
   * 
   * @param extend true = deploy, false = retract
   */
  public void setDeployed(boolean extend) {
    pistonExtended = extend;
    pneumaticDoubleSolenoidLeft.set(extend ? Value.kForward : Value.kReverse);
    pneumaticDoubleSolenoidRight.set(extend ? Value.kForward : Value.kReverse);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = deployed, false = retracted
   */
  public boolean isDeployed() {
    return pistonExtended;
  }

  /**
   * Toggles the piston between deploy and undeployed
   */
  public void toggleDeploy(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", isDeployed());
    setDeployed(!isDeployed());
  }
  

  // ************ Information methods

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
    // This method will be called once per scheduler run

    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateIntakeLog(false);
    }

    if(log.isMyLogRotation(logRotationKey)) {
      // Update data on SmartDashboard
      SmartDashboard.putNumber("Intake Motor 1 Amps", getMotor1Amps());
      SmartDashboard.putNumber("Intake Motor 1 Bus Volt", motor1.getBusVoltage());
      SmartDashboard.putNumber("Intake Motor 1 Out Percent", motor1.getMotorOutputPercent());
      SmartDashboard.putNumber("Intake Motor 2 Amps", getMotor2Amps());
      SmartDashboard.putNumber("Intake Motor 2 Bus Volt", motor2.getBusVoltage());
      SmartDashboard.putNumber("Intake Motor 2 Out Percent", motor2.getMotorOutputPercent());
      SmartDashboard.putBoolean("Intake Piston extend", pistonExtended);
    }
  }

  /**
   * Writes information about the Intake to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateIntakeLog(boolean logWhenDisabled){
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
    "Motor 1 Bus Volt", motor1.getBusVoltage(),
    "Motor 1 Out Percent", motor1.getMotorOutputPercent(),
    "Motor 1 Amps", getMotor1Amps(),
    "Motor 2 Bus Volt", motor2.getBusVoltage(),
    "Motor 2 Out Percent", motor2.getMotorOutputPercent(),
    "Motor 2 Amps", getMotor2Amps(),
    "Piston extended", isDeployed()
    );
  }

  
}
