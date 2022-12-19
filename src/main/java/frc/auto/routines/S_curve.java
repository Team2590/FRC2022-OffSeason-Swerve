package frc.auto.routines;

import frc.auto.AutoRoutine;
import frc.subsystems.DrivetrainSettings;


import frc.auto.AutoManager;

import frc.auto.trajectory.NemesisPath;
import frc.auto.trajectory.PathContainer;
import frc.robot.Robot;
import frc.subsystems.Drivetrain;


public class S_curve extends AutoRoutine implements DrivetrainSettings {
    Drivetrain d;
    NemesisPath S_path =PathContainer.S_curve;
    String pathname= "S_curve" ;


    public S_curve(){

    }

    @Override
    public String getKey() {
        // TODO Auto-generated method stub
        return pathname;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        d=Robot.getDrivetrainInstance();
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        // This is what the Robot is doing, which is running the path so the run path code goes here
        S_path.runPath(d);
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
        
    }

    }

    

