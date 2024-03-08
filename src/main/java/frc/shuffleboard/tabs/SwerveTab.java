package frc.shuffleboard.tabs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.shuffleboard.ShuffleboardTabBase;
import frc.team6014.lib.drivers.SwerveModuleBase;

public class SwerveTab extends ShuffleboardTabBase {

        private DriveSubsystem mDrive = DriveSubsystem.getInstance();

        private final SwerveModuleBase[] mDriveModules;

        private String[] kSwervePlacements = { "Front Left", "Front Right", "Back Left", "Back Right" };
        private ShuffleboardLayout[] mDriveLayouts = new ShuffleboardLayout[4];
        private GenericEntry[] mDriveCANCoders = new GenericEntry[4];
        private GenericEntry[] mDriveIntegrated = new GenericEntry[4];
        private GenericEntry[] mDriveMPS = new GenericEntry[4];

        private GenericEntry mDriveOdometryX;
        private GenericEntry mDriveOdometryY;
        private GenericEntry mDriveOdometryRot;

        public SwerveTab() {
                super();
                mDriveModules = mDrive.mSwerveModules;
        }

        @Override
        public void createEntries() {
                mTab = Shuffleboard.getTab("Swerve");

                mDriveOdometryX = mTab
                                .add("Odometry X", 0)
                                .withPosition(0, 3)
                                .withSize(2, 1)
                                .getEntry();
                mDriveOdometryY = mTab
                                .add("Odometry Y", 0)
                                .withPosition(2, 3)
                                .withSize(2, 1)
                                .getEntry();
                mDriveOdometryRot = mTab
                                .add("Pigeon Angle", 0)
                                .withPosition(4, 3)
                                .withSize(2, 1)
                                .withWidget(BuiltInWidgets.kGyro)
                                .getEntry();
        }

        @Override
        public void update() {
                for (int i = 0; i < mDriveCANCoders.length; i++) {
                        mDriveCANCoders[i].setDouble(truncate(mDriveModules[i].getCANCoderRotation().getDegrees()));
                        mDriveIntegrated[i]
                                        .setDouble(truncate(MathUtil.inputModulus(
                                                        mDriveModules[i].getState().angle.getDegrees(), 0, 360)));
                        mDriveMPS[i].setDouble(mDriveModules[i].getState().speedMetersPerSecond);
                }

                mDriveOdometryX.setDouble(truncate(mDrive.getPoseMeters().getX()));
                mDriveOdometryY.setDouble(truncate(mDrive.getPoseMeters().getY()));
                mDriveOdometryRot
                                .setDouble(truncate(
                                                MathUtil.inputModulus(mDrive.getRotation2d().getDegrees(), 0, 360)));

        }

}
