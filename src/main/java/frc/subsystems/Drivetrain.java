package frc.subsystems;
import frc.robot.RobotMap;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import java.util.function.DoubleSupplier;

public class Drivetrain implements RobotMap, Subsystem, DrivetrainSettings {

    private static Drivetrain driveTrainInstance = null;
    private double maxVoltage;
    private double maxVelocity; // Meters per second
    private double maxAngularVelocity; // Angular Velocity of the drivetrain in radians
    private AHRS gyro;
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private DoubleSupplier translationXSupp;
    private DoubleSupplier translationYSupp;
    private DoubleSupplier rotationSupp;

    private ChassisSpeeds driveSpeeds;
  
    public static Drivetrain getDriveInstance(PowerDistribution pdp) {
      if (driveTrainInstance == null) {
        driveTrainInstance = new Drivetrain(pdp);
      }
      return driveTrainInstance;
    }
    private enum States {
        STOPPED, DRIVE
      }
    private States driveState;
    private final SwerveDriveKinematics swerveKinematics;
    private SwerveModuleState[] states;

    
    public Drivetrain(PowerDistribution pdp) {
        
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        // constructor 
        maxVoltage = 12;
        maxVelocity = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        maxAngularVelocity = maxVelocity / 
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
        driveSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
        driveState = States.STOPPED;

        swerveKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );
        
    }
    public void update(){
        switch(driveState){
            case DRIVE:
                drive(driveSpeeds);
                break;
            
            case STOPPED:                
                drive(new ChassisSpeeds(0,0,0));
                break;
        }
    }
    public void inputHandler(double leftx, double lefty, double rightx) {
        driveState = States.DRIVE;
        double x = leftx * maxVelocity;
        double y = lefty * maxVelocity;
        double angle = rightx * maxAngularVelocity;
        driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, angle, gyro.getRotation2d());//jeevan and vidur contribution to this
       //driveSpeeds = new ChassisSpeeds(x, y, angle);
        System.out.println(driveSpeeds);
        System.out.printf("X: %f Y: %f Angle: %f\n",x,y,angle); 
    }
    public void drive(ChassisSpeeds speeds){
        states = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        // speedMetersPerSecond / maxVelocity == % of motor speed you wanna run 
        // speedMetersPerSecond is the speed you want to run this module at 
        // state has a velocity and angle 
        // normalize = scaling value down to [-1,1] 
        frontLeftModule.set((states[0].speedMetersPerSecond / maxVelocity)* maxVoltage, states[0].angle.getRadians());
        frontRightModule.set((states[1].speedMetersPerSecond / maxVelocity) * maxVoltage, states[1].angle.getRadians());
        backLeftModule.set((states[2].speedMetersPerSecond / maxVelocity) * maxVoltage, states[2].angle.getRadians());
        backRightModule.set((states[3].speedMetersPerSecond / maxVelocity) * maxVoltage, states[3].angle.getRadians());
        
    }
    public void zeroGyro() {
       gyro.zeroYaw();
      }
    public double getHeading() { 
        return gyro.getFusedHeading(); 
    }
    public Rotation2d getHeadingRot(){
        return new Rotation2d(getHeading());
    }
}
