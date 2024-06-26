// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */
  private NetworkTable tLL;
  private NetworkTableEntry tBotPose_field;
  private NetworkTableEntry tCamPose_target;
  private NetworkTableEntry tBotPose_target;
  private NetworkTableEntry tId;

  private double[] mBotPoseArray_field = new double[8];
  private double[] mCamPoseArray_target = new double[6];
  private double[] mBotPoseArray_target = new double[6];

  private Pose3d mBotPose3d_field;
  private Pose3d mCamPose3d_target;
  private Pose3d mBotPose3d_target;

  private Pose2d mBotPose2d_field;
  private Pose2d mCamPose2d_target;
  private Pose2d mBotPose2d_target;

  private int mId;

  private double mArea;
  private double mLatency;

  private static LimelightSubsystem mLL;

  public LimelightSubsystem() {
    updateValues();
  }

  @Override
  public void periodic() {
    updateValues();
    
  }

  private void updateValues() {
    tLL = NetworkTableInstance.getDefault().getTable(Constants.LLConstants.name);
    tBotPose_field = tLL.getEntry("botpose_wpiblue");
    tCamPose_target = tLL.getEntry("camerapose_targetspace");
    tBotPose_target = tLL.getEntry("botpose_targetspace");
    tId = tLL.getEntry("tid");

    mBotPoseArray_field = tBotPose_field.getDoubleArray(new double[7]);
    mBotPoseArray_target = tBotPose_target.getDoubleArray(new double[6]);
    mCamPoseArray_target = tCamPose_target.getDoubleArray(new double[6]);

    mBotPose2d_field = DoubleArrayToPose2d(mBotPoseArray_field);
    mBotPose2d_target = DoubleArrayToPose2d(mBotPoseArray_target);
    mCamPose2d_target = DoubleArrayToPose2d(mCamPoseArray_target);
    SmartDashboard.putString("LLL", mBotPose2d_field.toString());

    mBotPose3d_field = DoubleArrayToPose3d(mBotPoseArray_field);
    mBotPose3d_target = DoubleArrayToPose3d(mBotPoseArray_target);
    mCamPose3d_target = DoubleArrayToPose3d(mCamPoseArray_target);

    mId = (int) tId.getDouble(0);

    mArea = tLL.getEntry("ta").getDouble(0);

    mLatency = mBotPoseArray_field[6];
  }

  private static Pose3d DoubleArrayToPose3d(double[] inData) {
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  private static Pose2d DoubleArrayToPose2d(double[] inData) {
    return new Pose2d(
        new Translation2d(inData[0], inData[1]),
        new Rotation2d(Units.degreesToRadians(inData[5])));
  }

  public Pose3d getBotPose3d_field() {
    return mBotPose3d_field;
  }

  public Pose3d getBotPose3d_target() {
    return mBotPose3d_target;
  }

  public Pose3d getCamPose3d_target() {
    return mCamPose3d_target;
  }

  public Pose2d getBotPose2d_field() {
    return mBotPose2d_field;
  }

  public Pose2d getBotPose2d_target() {
    return mBotPose2d_target;
  }

  public Pose2d getCamPose2d_target() {
    return mCamPose2d_target;
  }

  public int getID() {
    return mId;
  }

  public int getNumTargets() {
    return (int) mBotPoseArray_field[7];
  }

  public double getArea() {
    return mArea;
  }

  public double getLatency() {
    return mLatency;
  }

  public static LimelightSubsystem getInstance() {
    if (mLL == null) {
      mLL = new LimelightSubsystem();
    }
    return mLL;
  }

}
