package frc.auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Drivetrain;

public class NemesisPath {
    private final Timer timer = new Timer();
    private boolean isStarted;
    private boolean isFinished;
    private Trajectory path= new Trajectory();
    public NemesisPath(TrajectoryConfig config, Path waypoints){
        isFinished = false;
        isStarted = false;
        
        try {
            path = TrajectoryUtil.fromPathweaverJson(waypoints);
            System.out.println("PATH INITIALIZED");
            
            
        } catch (IOException ex) {
            DriverStation.reportError("Path not Found" , ex.getStackTrace());
        }

         
    }
    public void runPath(Drivetrain drivetrain){
        if(!isStarted){
            // start it
            System.out.println("STARTING PATH+RESETTING ODOMETRY+ENCODERS");
            drivetrain.resetEncoder();

            drivetrain.resetOdometry(path.getInitialPose());
            drivetrain.outputOdometry();
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
            System.out.println(path.sample(currentTime));
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
