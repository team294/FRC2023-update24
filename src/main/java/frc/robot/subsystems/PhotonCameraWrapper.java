package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

import java.io.IOException;
import java.util.Optional;

// import javax.lang.model.util.Elements.Origin;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper extends SubsystemBase {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Field field;
  private FileLog log;
  private boolean hasInit = false;
  private Alliance currAlliance = Alliance.Invalid;
  private int logRotationKey;
  private boolean fastLogging = false;

  public PhotonCameraWrapper(Field field, FileLog log, int logRotationKey) {
    this.log = log;
    this.field = field;
    this.logRotationKey = logRotationKey;
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  public void enableFastLogging(boolean enabled) {
    this.fastLogging = enabled;
  }

  public void init() {
    log.writeLog(true, "PhotonCameraWrapper", "Init", "Starting");

    currAlliance = field.getAlliance();

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }


    //  aprilTagFieldLayout = field.getAprilTagFieldLayout();

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      currAlliance = field.getAlliance();
      switch (currAlliance) {
        case Blue:
          aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
          break;
        case Red:
          aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
          break;
        default:
          log.writeLog(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance invalid");
          break;
      }
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Loaded april tags from file");
    } catch (IOException e) {
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Error loading april tags from file");
      e.printStackTrace();
    }
    
    // Create pose estimator
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      photonCamera,
      VisionConstants.robotToCam);
      
    hasInit = true;

    log.writeLog(true, "PhotonCameraWrapper", "Init", "Done");
  }

  public boolean hasInit() {
    return hasInit;
  }

  public Alliance getAlliance() {
    return field.getAlliance();
  }

  public void periodic() {
    if (field.getAlliance() != currAlliance) {
      init();
      log.writeLogEcho(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance changed", currAlliance);
    }
  }

  /**
  * @param estimatedRobotPose The current best guess at robot pose
  * @return A pair of the fused camera observations to a single Pose2d on the
  *         field, and the time
  *         of the observation. Assumes a planar field and the robot is always
  *         firmly on the ground
  */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var newPoseOptional = photonPoseEstimator.update();
    if (newPoseOptional.isPresent()) {
      EstimatedRobotPose newPose = newPoseOptional.get();
      if(fastLogging || log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "PhotonCameraWrapper", "getEstimatedGlobalPose", "NewPose", "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
      }
    }
    return newPoseOptional;
  }
}