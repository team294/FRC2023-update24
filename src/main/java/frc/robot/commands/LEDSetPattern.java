/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSetPattern extends Command {
  private LED led;
  private FileLog log;
  private Color[][] pattern;
  private int strip; // which strip to set rainbow
  private double speed; // how often to shift rainbow, in seconds
  private Timer timer;
  private int patternNum = 0;
 
  /**
   * Set LED strip to a specified shifting pattern, with parameter seconds between each shift.
   * This command never ends.
   * <p> Note:  This command can run while the robot is disabled.
   * @param pattern pattern library to cycle though
   * @param strip strip number (0 or 1)
   * @param speed how often to shift rainbow, in seconds
   * @param led led strip (subsystem)
   */
  public LEDSetPattern(Color[][] pattern, int strip, double speed, LED led, FileLog log) {
    this.led = led;
    this.log = log;
    this.pattern = pattern;
    this.strip = strip;
    this.speed = speed;
    this.timer = new Timer();
    addRequirements(led);
  }

  /**
   * Set LED strip to a specified shifting pattern, with 0.5 seconds between each shift.
   * This command never ends.
   * <p> Note:  This command can run while the robot is disabled.
   * @param pattern pattern library to cycle through
   * @param strip strip number (0 or 1)
   * @param led led strip (subsystem)
   */
  public LEDSetPattern(Color[][] pattern, int strip, LED led, FileLog log) {
    this(pattern, strip, 0.5, led, log);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    timer.reset();
    timer.start();
    log.writeLog(false, "LEDSetPattern", "Init");
  }
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
	public void execute() {  
    if(patternNum > pattern.length - 1) patternNum = 0;
    
    led.setPattern(pattern[patternNum], 0.5, strip);
    
    if(timer.advanceIfElapsed(speed)) {
      patternNum++;
    }
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
	public boolean isFinished() {
      return false;
	}

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
