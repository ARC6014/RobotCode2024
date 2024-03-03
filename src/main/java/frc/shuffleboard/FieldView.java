package frc.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FieldView {
    private Field2d mField2d = new Field2d();
    private DriveSubsystem mSwerve = DriveSubsystem.getInstance();

    private Pose2d[] modulePoses = new Pose2d[4];
    private Pose2d robotPose = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        if (mSwerve.getPoseMeters() != null)
            robotPose = mSwerve.getPoseMeters();
        else
            robotPose = new Pose2d();

        SwerveModuleState[] moduleStates = mSwerve.getModuleStates();

        for (int i = 0; i < modulePoses.length; i++) {
            Translation2d updatedPosition = Constants.swerveModuleLocations[i]
                    .rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
            Rotation2d updatedRotation = mSwerve.getModuleStates()[i].angle.plus(robotPose.getRotation());
            if (moduleStates[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));
                ;
            }
            modulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();

        mField2d.setRobotPose(robotPose);
        addPose("Swerve Modules", modulePoses);
    }

    public void addPose(String name, Pose2d pose) {
        if (pose != null) {
            mField2d.getObject(name).setPose(pose);
        }
    }

    private void addPose(String name, Pose2d... pose) {
        if (pose != null) {
            mField2d.getObject(name).setPoses(pose);
        }
    }

    public void addTrajectory(String name, Trajectory traj) {
        if (traj != null) {
            mField2d.getObject(name).setTrajectory(traj);
        }
    }
}
