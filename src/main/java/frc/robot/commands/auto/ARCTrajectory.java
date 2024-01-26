package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ARCTrajectory {

    public DriveSubsystem swerve;

    public ARCTrajectory() {
        swerve = DriveSubsystem.getInstance();
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(swerve::getPoseMeters, swerve::resetOdometry, swerve::getChassisSpeed,
                swerve::setClosedLoopStates, Constants.holonomicPoseConfig, swerve::shouldAllianceFlipToRed, swerve);
        /* 
                AutoBuilder.configureHolonomic(swerve::getPoseMeters, swerve::resetOdometry, swerve::getChassisSpeed,
                swerve::setClosedLoopStates, Constants.holonomicPoseConfig, () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, swerve); */
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command getAutoPath() {
        return getAutoPath("Example Auto");
    }

}
