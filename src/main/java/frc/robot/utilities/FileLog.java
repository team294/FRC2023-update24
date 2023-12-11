/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import static frc.robot.utilities.StringUtil.*;

/**
 * Class used to write information to file for logging.
 *
 */
public class FileLog {
	
    private FileWriter fileWriter;
	private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
	private static final SimpleDateFormat fileDateFormat = new SimpleDateFormat("yyyy-MM-dd.HH-mm-ss");
	private String fileNameBase, fileNameFull;
	private long startTime;
	private int logLevel = 3; // Level of detail. Value between 1-3, where 1 is the most detailed and 3 is the least detailed.
    
    // File logging rotation cycles, to spread out logging times between subsystems	
	private final int NUM_ROTATIONS = 10;
	private int rotationLastAllocated = NUM_ROTATIONS-1;
	private int rotationCurrent = 0;			// Values = 0 .. NUM_ROTATIONS-1
    
	/**
	 * Creates a new log file called "/home/lvuser/logfile.ver.date.time.csv"
	 * When running a robot simulation on a laptop, the log file is placed in the VSCode project in the "sim" subdirectory
     * @param version Version of robot code (i.e. A1).
	 */
	public FileLog(String version) {
		// When running on a RoboRio:  Logfile is placed in /home/lvuser/
		// When running in simulation:  Logfile is placed in the VSCode project in the "sim" subdirectory
		// 		Note that the "sim" directory must exist, otherwise the code will crash when simulating.
		//		TODO when simulating, check if the sim directory exists, and if not then create it.
		this(RobotBase.isReal() ? "/home/lvuser/logfile" : "sim\\logfilesim", version);
	}
	
	/**
	 * Creates a new log file. ".ver.date.time.csv" will automatically be added to the end of the base file name.
	 * @param filenameBase Path and name of log file.
     * @param version Version of robot code (i.e. A1).
	 */
	public FileLog(String filenameBase, String version) {
		this.fileNameBase = buildString(filenameBase, ".", version, ".");
        startTime = System.currentTimeMillis();
		fileNameFull = buildString(fileNameBase, (fileDateFormat.format(startTime)), ".csv");

		try {
			fileWriter = new FileWriter(fileNameFull, true);
			fileWriter.write("----------------------------\n");
			fileWriter.write(buildString(dateFormat.format(System.currentTimeMillis()), ",FileLog,Open,", fileNameFull, "\n"));
			fileWriter.flush();
		} catch (IOException exception) {
			System.out.println("Could not open log file: " + exception);
        }	
    }
    
    /**
	 * Renames the log file name using the current date and time
	 */
	public void updateFilenameDateTime() {
		String fileNameNew;
		File oldFile, newFile;

		// Close the current log file
		try {
			fileWriter.close();
		} catch (IOException exception) {
		}

		// Update startTime and generate the new file name
		startTime = System.currentTimeMillis();
		fileNameNew = buildString(fileNameBase, (fileDateFormat.format(startTime)), ".csv");

		// Rename the file
		oldFile = new File(fileNameFull);
		newFile = new File(fileNameNew);
		oldFile.renameTo(newFile);

		// Update member variables and open the new file
		fileNameFull = fileNameNew;
		try {
			fileWriter = new FileWriter(fileNameFull, true);
			fileWriter.write("----------------------------\n");
			fileWriter.write(buildString((dateFormat.format(System.currentTimeMillis())), ",FileLog,Rename,", fileNameFull, "\n"));
            fileWriter.flush();                
		} catch (IOException exception) {
			System.out.println("Could not open log file: " + exception);
		}
	}   

    /**
	 * Writes a message to the log file. The message will be timestamped. Does not echo the message to the screen.
	 * @param logWhenDisabled true will log when disabled, false will discard the message
     * @param subsystemOrCommand The name of the subsystem or command generating the message.
	 * @param event A description of the event (ex. start, data, event).
	 * @param paramArray... List of descriptions and values (variable number of parameters)
	 */
	public void writeLog(boolean logWhenDisabled, String subsystemOrCommand, String event, Object... paramArray) {
        // If system clock has reset by more than 24 hours (like when the clock is set
        // at the start of a match), then fix the filename.
		if (System.currentTimeMillis() - startTime > 1000*3600*24) {
			updateFilenameDateTime();
		}

		// Write the message to the file.
		if(logWhenDisabled || DriverStation.isEnabled()) {
            try {
			fileWriter.write(buildStringWithCommas((dateFormat.format(System.currentTimeMillis())), subsystemOrCommand, event, buildStringWithCommas((Object [])paramArray).concat("\n")));
            fileWriter.flush();
		    } catch (IOException exception) {
            }
		}
	}
	
    /**
	 * Writes a message to the log file. The message will be timestamped. Does not echo the message to the screen.
	 * @param logWhenDisabled true will log when disabled, false will discard the message
     * @param subsystemOrCommand The name of the subsystem or command generating the message.
	 * @param event A description of the event (ex. start, data, event).
	 * @param paramArray... List of descriptions and values (variable number of parameters)
	 */
	public void writeLogEcho(boolean logWhenDisabled, String subsystemOrCommand, String event, Object... paramArray) {
		writeLog(logWhenDisabled, subsystemOrCommand, event, (Object [])paramArray);
		System.out.println(buildStringWithCommas("Log", subsystemOrCommand, event, buildStringWithCommas((Object [])paramArray)));
	}
	
    /**
	 * Changes level of detail for fileLog
	 * Level 1 = full debugging logs.  Huge file log, so use sparingly.
	 * Level 2 = normal lab mode.  Moderate logging details.
	 * Level 3 = competition mode.  Minimal logging.
	 * @param level between 1-3, where 1 is the most detailed and 3 is the least detailed.
	 */
	public void setLogLevel(int level) {
		logLevel = level;
		writeLogEcho(true, "FileLog", "setLogLevel", "Level", level);
	}

	/**
	 * Returns what level of detail the fileLog should be at (to be called in each subsystem)
	 * Level 1 = full debugging logs.  Huge file log, so use sparingly.
	 * Level 2 = normal lab mode.  Moderate logging details.
	 * Level 3 = competition mode.  Minimal logging.
	 */
	public int getLogLevel() {
		return logLevel;
	}


	/** 
	 * Advances the log rotation counter by one place, and resets if above threshhold for the current log level
	 */
	public void advanceLogRotation() {
		rotationCurrent++;
		if (rotationCurrent >= NUM_ROTATIONS) rotationCurrent = 0;
		// if (logLevel == 3) {
		// 	if (rotation >= 25) rotation = 0;
		// } else {
		// 	if (rotation >= 10) rotation = 0;
		// } TODO add back in when we do log levels
	}

	/**
	 * Gets the index of the file log rotation
	 * @return int between 0 and 1 (log levels 1 or 2) or between 0 and 24 (log level 3)
	 */
	public int getLogRotation() {
		return rotationCurrent;
	}

	/**
	 * Allocates a rotation index to a subsystem.  Use the returned value (logRotationKey) with
	 * isMyLogRotation(logRotationKey) to see if this is the current rotation for a given
	 * subsystem.
	 * @return allocated index for this subsystem
	 */
	public int allocateLogRotation() {
		rotationLastAllocated++;
		rotationLastAllocated %= NUM_ROTATIONS;
		return rotationLastAllocated;
	}

	/**
	 * Returns true if the scheduler is currently at rotationKey
	 * @param logRotationKey Key from allocateLogRotation() to check
	 * @return true = rotationKey is the current rotation, false = rotationKey is not the
	 * current rotation
	 */
	public boolean isMyLogRotation(int logRotationKey) {
		return (logRotationKey==rotationCurrent);
	}

	/**
	 * Closes the log file.  All writes after closing the log file will be ignored.
	 */
	public void close() {
		try {
			fileWriter.close();
			fileWriter = null;
		} catch (IOException exception) {
		}
	}
}