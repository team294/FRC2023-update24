/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

/******************************
 *  import frc.robot.utilities.ColorSensor;
 *  All references to the color sensor are commented out, since as of now we are not using one.
 *****************************/

import static edu.wpi.first.wpilibj.util.Color.*;

import edu.wpi.first.math.MathUtil;

public class LED extends SubsystemBase {
  private AddressableLED led; // led strip
  private AddressableLEDBuffer ledBuffer; // data passed to strip
  private final int length = 32; // length of strip, in pixels
  private final int firstStripLength = 32;
  private int startingInd = 0;
  // private boolean runAnimation = false;
  // private int currPattern = -1;
  // private String prevColor, currColor; // colors on the control panel
  // private final ColorSensor colorSensor; // reference to the color sensor

  /**
   * Color library for tracking the vision target.
   */
  public static final Color[][] visionTargetLibrary = {
    {kRed, kRed, kRed, kRed, kRed, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kRed, kRed, kRed, kRed, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kRed, kRed, kRed, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kRed, kRed, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kRed, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kRed, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kRed, kBlue, kBlue, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kGreen, kGreen, kGreen, kGreen, kGreen, kGreen, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kRed, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kRed, kRed, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kRed, kRed, kRed, kBlack, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kRed, kRed, kRed, kRed, kBlack},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlue, kBlue, kRed, kRed, kRed, kRed, kRed, kRed, kRed},
    {kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow}
  };

  /**
   * Color library for counting power cells shot.
   */
  public static final Color[][] powerCellsLibrary = {
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
    {kBlack, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack, kYellow, kYellow, kYellow, kYellow, kBlack},
  };

  /**
   * Color library for rainbow pattern.
   */
  public static final Color[][] rainbowLibrary = {
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue},
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue},
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue},
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue},
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue},
    {kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple},
    {kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed},
    {kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange},
    {kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow},
    {kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen},
    {kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue, kGreen, kYellow, kOrange, kRed, kPurple, kBlue}
  };

  public static final Color[][] teamFullColorsLibrary = {
    {kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue, kBlue},
    {kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed}
  };

  public static final Color[][] teamFlashingColorsLibrary = {
    {kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed},
    {kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue, kOrangeRed, kBlue}
  };

  public static final Color[][] teamMovingColorsLibrary = {
    {kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed},
    {kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed},
    {kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue},
    {kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue, kBlue, kOrangeRed, kOrangeRed, kBlue}
  };

  public static final Color[][] flashingRed = {
    {kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed},
    {kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack, kBlack},
  };
  
 
  /**
   * Controls LED strips on without the ColorSensor.
   */
  public LED () {
    led = new AddressableLED(Ports.PWMLEDStripTop);        // must be a PWM port, currently port 0
    ledBuffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    /** 
    // save reference to the color sensor
    this.colorSensor = new ColorSensor();

    // set prev and curr Color to see if color is changing
    prevColor = colorSensor.getColor();
    currColor = colorSensor.getColor();

    setColor(Color2.kBlack, 0);
**/
  }
 
  /**
   * Set color of individual pixel on LED strip.
   * @param index index of pixel to be changed
   * @param intensity multiplier for brightness (0 to 1)
   * @param color desired LED color (ex: kBlue or kRed)
   */
  public void setColor(int index, double intensity, Color color) {
    intensity = MathUtil.clamp(intensity, 0.0, 1.0);

    ledBuffer.setRGB(index, (int)(255*intensity * color.red), (int)(255*intensity * color.green), (int)(255*intensity * color.blue));
  }

  /**
   * Sets color of individual pixel on LED strip.
   * @param index index of pixel to be changed
   * @param r red value
   * @param b blue value
   * @param g green value
   */
  public void setColor(int index, int r, int b, int g) {
    ledBuffer.setRGB(index, r, g, b);
  }

  /**
   * Sets entire strip to be one color at 0.5 brightness
   * @param color String name of color, case sensitive
   * @param ledStrip (int) chooses which led strip you use 0 = first 1 = second
   */
  public void setStrip(Color color, int ledStrip) {
    if(ledStrip == 0){
      startingInd = 0;
    } else {
      startingInd = firstStripLength;
    }
    setStrip(color, 0.5, startingInd);
  }

  /**
   * Sets entire strip to be one color
   * @param color String name of color, case sensitive
   * @param brightness is the multiplier for brightness (0.5 is half)
   * @param ledStrip (int) chooses which led strip you use 0 = first 1 = second 
   */
  public void setStrip(Color color, double brightness, int ledStrip) {
    int myLength = length;
    if(ledStrip == 0){
      startingInd = 0;
      myLength = firstStripLength;
    } else {
      startingInd = firstStripLength;
    }
    for (int i = startingInd; i < myLength; i++) {
      setColor(i, brightness, color);
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Light up the LED strip in a solid color.
   * @param color Color2 to set to the entire strip
   * @param intensity 0 to 1
   * @param ledStrip (int) chooses which led strip you use 0 = first 1 = second
   */
  public void setColor(Color color, double intensity, int ledStrip) {
    int myLength = length;
    if(ledStrip == 0){
      startingInd = 0;
      myLength = firstStripLength;
    } else {
      startingInd = firstStripLength;
    }
    // int r, g, b;
    for(int l = startingInd; l < myLength; l++){
      ledBuffer.setRGB(l, (int)(255*intensity*color.red), (int)(255*intensity*color.green), (int)(255*intensity*color.blue));
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Light up the LED strip in a pattern.
   * @param pattern 1D array of Color values for the pattern
   * @param intensity 0 to 1
   * @param ledStrip (int) chooses which led strip you use 0 = first 1 = second
   */
  public void setPattern(Color[] pattern, double intensity, int ledStrip) {
    // int r, g, b;
    if(ledStrip == 0){
      startingInd = 0;

    } else {
      startingInd = firstStripLength;
    }
    int pixel = 0;
    for(int l = startingInd; l < pattern.length + startingInd; l++){
      ledBuffer.setRGB(l, (int)(255*intensity*pattern[pixel].red), (int)(255*intensity*pattern[pixel].green), (int)(255*intensity*pattern[pixel].blue));
      pixel++;
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * turns strip on or off
   * @param on true = on, false = off
   */
  public void setStripState(boolean on) {
    
    if (on) led.start();
    else led.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
/*************************************************************** 
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    if (colorSensor.getProximity() <1000) {
    
      prevColor = currColor;
      currColor = colorSensor.getColor();

      // change color of LED Strip if ColorSensor reads a new color
      if (!prevColor.equals(currColor)) setStrip(currColor);
      
      // ColorSensor updated values on dashboard
      SmartDashboard.putBoolean("Yellow", currColor == ("Yellow"));
      SmartDashboard.putBoolean("Red", (currColor=="Red"));
      SmartDashboard.putBoolean("Green", (currColor == "Green"));
      SmartDashboard.putBoolean("Blue", (currColor == "Blue"));
      SmartDashboard.putString("Color", currColor);
    }
*********************************************************/
  
  }
}
