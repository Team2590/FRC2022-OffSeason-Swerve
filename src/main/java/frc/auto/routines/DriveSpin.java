package frc.auto.routines;

import frc.auto.AutoManager;
import frc.auto.AutoRoutine;
import frc.auto.trajectory.NemesisPath;
import frc.auto.trajectory.PathContainer;
import frc.robot.Robot;
import frc.subsystems.Drivetrain;
import frc.subsystems.DrivetrainSettings;

public class DriveSpin extends AutoRoutine implements DrivetrainSettings{
    
    Drivetrain driveT;

    public DriveSpin(){

    }

    @Override
    public String getKey() {
        // TODO Auto-generated method stub
        return "DriveSpin";
    }

    @Override
    public void initialize() {
        System.out.println("INITALIAIRD");
        // TODO Auto-generated method stub
        driveT = Robot.getDrivetrainInstance();
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
      
        // System.out.println("running auto");
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
    }

}
