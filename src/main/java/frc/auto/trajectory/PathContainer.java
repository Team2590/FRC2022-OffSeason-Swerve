package frc.auto.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.subsystems.DrivetrainSettings;

public class PathContainer implements DrivetrainSettings{
    // trajectory config should be the same for all paths
    static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(3, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveKinematics);
        // Apply the voltage constraint
        // .addConstraint(autoVoltageConstraint);

    TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(3, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveKinematics)
        .setReversed(true);
        // Apply the voltage constraint
        // .addConstraint(autoVoltageConstraint);

        // Below will instantiate new paths 
        public static NemesisPath moveForward = new NemesisPath(TRAJECTORY_CONFIG, 
            new Pose2d(0,0, new Rotation2d()), 
            new Pose2d(0.4,0, new Rotation2d()), 
            new Translation2d[0]  
        );
}
