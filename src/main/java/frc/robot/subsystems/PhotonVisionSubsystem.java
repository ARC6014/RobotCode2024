// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PVConstants;

public class PhotonVisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(PVConstants.PV_CAMERA_NAME);
  private PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;
  Pose3d cameraPosOnRobot = new Pose3d(
      new Translation3d(PVConstants.CAM_POSE[0][0], PVConstants.CAM_POSE[0][1], PVConstants.CAM_POSE[0][2]),
      new Rotation3d(PVConstants.CAM_POSE[1][0], PVConstants.CAM_POSE[1][1], PVConstants.CAM_POSE[1][2]));

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    result = camera.getLatestResult();
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    if (hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
    }
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public PhotonPipelineResult getLatestResult() {
    return result;
  }

  public List<PhotonTrackedTarget> getTargets() {
    return targets;
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public Transform3d getCameraToTarget() {
    return bestTarget.getBestCameraToTarget();

  }

  public Pose3d getRobotPoseOnField() {
    return PhotonUtils.estimateFieldToRobotAprilTag(getCameraToTarget(),
        getCameraToRobot(),
        new Transform3d(cameraPosOnRobot.getTranslation(), cameraPosOnRobot.getRotation()));
  }

  public Pose3d getCameraToRobot() {
    try {
      var targetPose = new AprilTagFieldLayout(PVConstants.APRIL_TAG_LAYOUT).getTagPose(getTargetID());
      if (targetPose.isPresent()) {
        return targetPose.get();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    return new Pose3d();
  }

  public double getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getDistanceToPose(robotPose, targetPose);
  }

  public Rotation2d getTheYawToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getYawToPose(robotPose, targetPose);
  }

  public Translation2d getTranslationToTarget(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.estimateCameraToTargetTranslation(getDistanceToPose(robotPose, targetPose),
        Rotation2d.fromDegrees(-bestTarget.getYaw()));
  }

  /*
   * meters
   */
  public double getDistanceToTarget(Pose2d targetPose2d) {
    return PhotonUtils.calculateDistanceToTargetMeters(cameraPosOnRobot.getZ(),
        targetPose2d.getY(), cameraPosOnRobot.getRotation().getZ(),
        Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  public void takeSnapshot() {
    camera.takeOutputSnapshot();
  }

  public int getTargetID() {
    return bestTarget.getFiducialId();
  }

  public double getPoseAmbiguity() {
    return bestTarget.getPoseAmbiguity();
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }
}
