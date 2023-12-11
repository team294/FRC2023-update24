/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class LEDSetStrip extends Command {
  private LED led;
  private FileLog log;
  private Color color;
  private double intensity;
 
  /**
   * Send a solid color to the LED strip, at 0.5 intensity.
   * This command immediately finishes.
   * <p> Note:  This command can run while the robot is disabled.
   * @param color string of color name, case sensitive (ex: "Blue")
   * @param led led strip (subsystem)
   **/
	public LEDSetStrip(Color color, LED led, FileLog log) {
    this(color, 0.5, led, log);
  }
  
  /**
   * Send a solid color to the LED strip, at parameter intensity.
   * This command immediately finishes.
   * <p> Note:  This command can run while the robot is disabled.
   * @param color string of color name (first letter capital, ex: "Blue")
   * @param intensity percent intensity (0 to 1)
   * @param led led strip (subsystem)
   **/
	public LEDSetStrip(Color color, double intensity, LED led, FileLog log) {
    this.led = led;
    this.log = log;
    this.color = color;
    this.intensity = intensity;
	  addRequirements(led);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    if (intensity >= 0) led.setStrip(color, intensity, 0);
    else led.setStrip(color, 0);
    log.writeLog(false, "LEDSetStrip", "Init", "SetColor", color);
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
	public void execute() {  
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
	public boolean isFinished() {
		return true;
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
