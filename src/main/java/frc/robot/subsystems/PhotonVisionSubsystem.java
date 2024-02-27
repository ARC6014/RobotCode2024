// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PVConstants;

public class PhotonVisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(PVConstants.PV_CAMERA_NAME);
  private PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;
  int targetID = 0;
  double poseAmbiguity = 0.0;
  Transform3d bestCameraToTarget;

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
      targetID = bestTarget.getFiducialId();
      poseAmbiguity = bestTarget.getPoseAmbiguity();
      bestCameraToTarget = bestTarget.getBestCameraToTarget();
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
    return bestCameraToTarget;
  }

  public void takeSnapshot() {
    camera.takeOutputSnapshot();
  }

  public int getTargetID() {
    return targetID;
  }

  public double getPoseAmbiguity() {
    return poseAmbiguity;
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }
}
