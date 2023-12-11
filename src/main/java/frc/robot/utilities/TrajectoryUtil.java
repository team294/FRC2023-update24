package frc.robot.utilities;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * Utility class for Trajectories
 */
public class TrajectoryUtil {
		
	/**
	 * Dump the trajectory to the log
	 */  
 	public static void dumpTrajectory(Trajectory trajectory, FileLog log) {
		for (State s : trajectory.getStates()) {
			var pose = s.poseMeters;
			var translation = pose.getTranslation();
			var rotation = pose.getRotation();

			log.writeLog(true, "TrajectoryGeneration", "Trajectory", 
				"Time", s.timeSeconds, 
				"x", translation.getX(), 
				"y", translation.getY(), 
				"degrees", rotation.getDegrees(), 
				"velocityMetersPerSec", s.velocityMetersPerSecond);
			
		}
	}

}
