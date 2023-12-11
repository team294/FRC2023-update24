/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * Utility with a dumb wait loop
 */
public class Wait {
    
    /**
     * Dumb loop that waits the time specified.
     * <p>NOTE:  Do not use this except in RobotInit or Subsystem constructors!!!!
     * @param millis how long to wait, in milliseconds
     */
    public static void waitTime(long millis) {
        long t = System.currentTimeMillis() + millis;

        while (System.currentTimeMillis() < t);
    }
}
