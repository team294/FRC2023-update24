package frc.robot.utilities;

import com.choreo.lib.ChoreoControlFunction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveTrain;

public class BCRChoreoController {
    
    /** 
     * This function is copied from choreoSwerveController function with added file logging
     * <p>
     * Creates a control function for following a ChoreoTrajectoryState.
     * @param xController A PIDController for field-relative X translation (input: X error in meters,
     *     output: m/s).
     * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
     *     output: m/s).
     * @param rotationController A PIDController for robot rotation (input: heading error in radians,
     *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
     *     ChoreoLib.
     * @param driveTrain A drive train subsystem
     * @param log A file log subsystem
     * @return A ChoreoControlFunction to track ChoreoTrajectoryStates. This function returns
     *     robot-relative ChassisSpeeds.
     */
    public static ChoreoControlFunction BCRChoreoSwerveController(
        PIDController xController, PIDController yController, PIDController rotationController, DriveTrain driveTrain, FileLog log){
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            return (pose, referenceState) -> {
                double xFF = referenceState.velocityX;
                double yFF = referenceState.velocityY;
                double rotationFF = referenceState.angularVelocity;

                double xFeedback = xController.calculate(pose.getX(), referenceState.x);
                double yFeedback = yController.calculate(pose.getY(), referenceState.y);
                double rotationFeedback =
                    rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

                ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
                log.writeLog(false, "BCRChoreoController", "Apply", 
                "Time", referenceState.timestamp,
                "Traj X", referenceState.x,
                "Traj Y", referenceState.y,
                "Traj Vel", Math.hypot(referenceState.velocityX, referenceState.velocityY),
                "Traj VelAng", referenceState.angularVelocity,
                "Target rot", referenceState.heading,
                "Robot X", pose.getTranslation().getX(),
                "Robot Y", pose.getTranslation().getY(),
                "Robot Vel", Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond),
                "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
                "Robot rot", pose.getRotation().getDegrees());

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
            };
        }
}
