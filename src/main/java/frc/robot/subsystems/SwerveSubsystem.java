// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


// import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveModule frontLeft = new SwerveModule(
    Constants.FrontLeft.kDriveMotorID, 
    Constants.FrontLeft.kTurnMotorID, 
    Constants.FrontLeft.kDriveMotorReversed, 
    Constants.FrontLeft.kTurningMotorReversed, 
    Constants.FrontLeft.kAbsoluteEncoderID);
  
  private final SwerveModule frontRight = new SwerveModule(
    Constants.FrontRight.kDriveMotorID, 
    Constants.FrontRight.kTurnMotorID, 
    Constants.FrontRight.kDriveMotorReversed, 
    Constants.FrontRight.kTurningMotorReversed, 
    Constants.FrontRight.kAbsoluteEncoderID);

  private final SwerveModule backLeft = new SwerveModule(
    Constants.BackLeft.kDriveMotorID, 
    Constants.BackLeft.kTurnMotorID, 
    Constants.BackLeft.kDriveMotorReversed, 
    Constants.BackLeft.kTurningMotorReversed, 
    Constants.BackLeft.kAbsoluteEncoderID);
  
  private final SwerveModule backRight = new SwerveModule(
    Constants.BackRight.kDriveMotorID, 
    Constants.BackRight.kTurnMotorID, 
    Constants.BackRight.kDriveMotorReversed, 
    Constants.BackRight.kTurningMotorReversed, 
    Constants.BackRight.kAbsoluteEncoderID);

  // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.RobotStructure.kDriveKinematics, new Rotation2d(0), 
    getPositions());

  public void zeroHeading(){
    gyro.reset();



  //path planner
  // AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getCurrentSpeeds, this::drive, Constants.PathPlannerConstants.kHolonomicPathFollowerConfig, 
  //   () -> {
  //             // Boolean supplier that controls when the path will be mirrored for the red alliance
  //             // This will flip the path being followed to the red side of the field.
  //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  //             return true;

  //             //var alliance = DriverStation.getAlliance();
  //             //if (alliance.isPresent()) {
  //             //  return alliance.get() == DriverStation.Alliance.Red;
  //             //}
  //             //return false;
  //         },
  //    this);
  // }

    RobotConfig config = (RobotConfig) null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getCurrentSpeeds, 
      (speeds, feedforwards) -> drive(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(5,0,0),
        new PIDConstants(5,0,0)
      ),
      config, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return true;
        
        //var alliance = DriverStation.getAlliance();
        //if (alliance.isPresent()) {
        //  return alliance.get() == DriverStation.Alliance.Red;
        //}
        //return false;
      },
      this
    );

  // public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
  //           new PIDConstants(5,0,0),
  //           new PIDConstants(5,0,0),
  //           DriveConstants.kPhysicalMaxSpeedMPS,
  //           0, //max distance to wheel
  //           new ReplanningConfig());    
  
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {

      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}

    }).start();

  
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
    odometer.resetPosition(getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return Constants.RobotStructure.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    setModuleStates(Constants.RobotStructure.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }


  public SwerveModulePosition[] getPositions(){
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()};
  }



  @Override
  public void periodic(){

    odometer.update(getRotation2d(), getPositions());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kPhysicalMaxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);;
    frontRight.setDesiredState(desiredStates[1]);;
    backLeft.setDesiredState(desiredStates[2]);;
    backRight.setDesiredState(desiredStates[3]);;

  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = {frontLeft.getState(), backLeft.getState(), backRight.getState(), frontRight.getState()};
    return states;
  }


}
