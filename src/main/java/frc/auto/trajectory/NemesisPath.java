package frc.auto.trajectory;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Drivetrain;

public class NemesisPath {
    private final Timer timer = new Timer();
    private boolean isStarted;
    private boolean isFinished;
    private Trajectory path;
    public NemesisPath(TrajectoryConfig config, Pose2d initialPose, Pose2d finalPose2d, Translation2d[] waypoints){
        isFinished = false;
        isStarted = false;
        path = TrajectoryGenerator.generateTrajectory(initialPose, List.of(waypoints), finalPose2d, config); 
    }
    public void runPath(Drivetrain drivetrain){
        if(!isStarted){
            // start it
            System.out.println("STARTING PATH");
            timer.reset();
            timer.start();
            isStarted = true;
        } 
        else if(isStarted) {
            // follow the path. Send command to drivetrain
            double currentTime = timer.get();
            System.out.println("TIME: " + currentTime);
            System.out.println("SAMPLE:");
            System.out.println(path.sample(currentTime));
            System.out.println(drivetrain);
            drivetrain.followPath(path.sample(currentTime));   
        }
    }
    public boolean getIsStarted(){
        return isStarted;
    }
    public boolean getIsFinished(){
        return isFinished;
    }
    public void setWait(long duration){
        try{
            path.wait(duration);
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
