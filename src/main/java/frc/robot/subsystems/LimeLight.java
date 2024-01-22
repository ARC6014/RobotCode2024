// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */

  private double camX;//meters
  private double camY;
  private double camZ;
  private double camTethaRad;
  private double camTethaDeg;
  private Pose3d camPose3d;
  private Pose2d camPose2d;
  private double tagID;

  private static String mLLname = Constants.LLConstants.name;
  private static double mheight = Constants.LLConstants.height;
  private static double mPitch = Constants.LLConstants.Pitch;

  private static LimeLight mLL;

  public LimeLight() {
    updateValues();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();

    SmartDashboard.putNumber("LL X", camX);
    SmartDashboard.putNumber("LL Y", camY);
    SmartDashboard.putNumber("LL Z", camZ);
    SmartDashboard.putNumber("LL Theta", (camTethaRad/Math.PI)*180);
    SmartDashboard.putNumber("Tag ID", tagID);
    
  }

  public void updateValues(){
    tagID = LimelightHelpers.getFiducialID("limelight");
    camPose3d = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
    camPose2d = camPose3d.toPose2d();
    camX = camPose3d.getX();
    camY = camPose3d.getY();
    camZ = camPose3d.getZ();
    camTethaRad = camPose3d.getRotation().getZ(); // roll(x) should be 0, pitch(y) should be fixed
    camTethaDeg = (camTethaRad/Math.PI) * 180;
  }

  public Pose3d getCamPose3d(){
    updateValues();
    return camPose3d;
  }

  public Pose2d getCamPose2d(){
    updateValues();
    return camPose2d;
  }

  public double getCamX(){
    updateValues();
    return camX;
  }

  public double getCamY(){
    updateValues();
    return camY;
  }

  public double getCamZ(){
    updateValues();
    return camZ;
  }
  
  public double getCamThetaRad(){
    updateValues();
    return camTethaRad;
  }

  public double getCamThetaDeg(){
    updateValues();
    return camTethaDeg;
  }

  public double getTagID(){
    updateValues();
    return tagID;
  }

  public static LimeLight getInstance() {
    if (mLL == null) {
      mLL = new LimeLight();
    }
    return mLL;
  }

  
}

