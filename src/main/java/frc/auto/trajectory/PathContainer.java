package frc.auto.trajectory;

import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import frc.subsystems.DrivetrainSettings;

public class PathContainer implements DrivetrainSettings{
    // trajectory config should be the same for all paths
    static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(3, 1).setKinematics(swerveKinematics); //.addConstraint(autoVoltageConstraint);
        // Add kinematics to ensure max speed is actually obeyed
       
        // Apply the voltage constraint
    
    
    //new stuff for the S curve swerve path
    static Pose2d initPos=new Pose2d(0,0,new Rotation2d(0));
    static Pose2d finalPos=new Pose2d(3,3, new Rotation2d(180));
   
   
    static Translation2d[] S_array =new Translation2d[]{new Translation2d(1,1),new Translation2d(1,1.5)};
    static String RoboRios_curveJSOn="paths/S curve part 1.wpilib.json";
   
   
   
   
    static Path s_curveJSon=Path.of("C:\\Users\\ayan\\OffseasonSwerve\\PathWeaver\\output\\S curve part 1.wpilib.json");//This gets the local path
    static Path moveForwardJSon=Path.of("C:\\Users\\ayan\\OffseasonSwerve\\PathWeaver\\output\\MoveForward");
    static Path s_curvePath=Filesystem.getDeployDirectory().toPath().resolve(RoboRios_curveJSOn); //This gets the path from the src/main/deploy

       
    



        

    TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(3, 1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveKinematics)
        .setReversed(true);



        public static NemesisPath S_curve= new NemesisPath(TRAJECTORY_CONFIG, s_curvePath);



         //public static NemesisPath S_curve=NemesisPath(TRAJECTORY_CONFIG, s_curveJSon)
        // Apply the voltage constraint
        // .addConstraint(autoVoltageConstraint);

        // Below will instantiate new paths 
       public static NemesisPath moveForward = new NemesisPath(TRAJECTORY_CONFIG, moveForwardJSon);
      
        
        /*
        public static NemesisPath turnAround=new NemesisPath(TRAJECTORY_CONFIG , initPos, finalPos, S_array); */
    
    }

