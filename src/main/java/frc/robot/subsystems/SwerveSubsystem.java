// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {

  private Field2d field = new Field2d();

  private final SwerveModule frontLeft = new SwerveModule(
      Constants.FrontLeft.DRIVE_MOTOR_ID,
      Constants.FrontLeft.TURN_MOTOR_ID,
      Constants.FrontLeft.DRIVE_MOTOR_REVERSED,
      Constants.FrontLeft.TURNING_MOTOR_REVERSED,
      Constants.FrontLeft.ABSOLUTE_ENCODER_ID,
      Constants.FrontLeft.ENCODER_OFFSET);

  private final SwerveModule frontRight = new SwerveModule(
      Constants.FrontRight.DRIVE_MOTOR_ID,
      Constants.FrontRight.TURN_MOTOR_ID,
      Constants.FrontRight.DRIVE_MOTOR_REVERSED,
      Constants.FrontRight.TURNING_MOTOR_REVERSED,
      Constants.FrontRight.ABSOLUTE_ENCODER_ID,
      Constants.FrontRight.ENCODER_OFFSET);

  private final SwerveModule backLeft = new SwerveModule(
      Constants.BackLeft.DRIVE_MOTOR_ID,
      Constants.BackLeft.TURN_MOTOR_ID,
      Constants.BackLeft.DRIVE_MOTOR_REVERSED,
      Constants.BackLeft.TURNING_MOTOR_REVERSED,
      Constants.BackLeft.ABSOLUTE_ENCODER_ID,
      Constants.BackLeft.ENCODER_OFFSET);

  private final SwerveModule backRight = new SwerveModule(
      Constants.BackRight.DRIVE_MOTOR_ID,
      Constants.BackRight.TURN_MOTOR_ID,
      Constants.BackRight.DRIVE_MOTOR_REVERSED,
      Constants.BackRight.TURNING_MOTOR_REVERSED,
      Constants.BackRight.ABSOLUTE_ENCODER_ID,
      Constants.BackRight.ENCODER_OFFSET);

  // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  public final AHRS zeppeli = new AHRS(NavXComType.kMXP_SPI);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.RobotStructure.DRIVE_KINEMATICS,
      new Rotation2d(0),
      getPositions());// , new Pose2d(10, 6, Rotation2d.fromDegrees(0)));

  public SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      Constants.RobotStructure.DRIVE_KINEMATICS,
      new Rotation2d(0),
      getPositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private RobotConfig config = null;

  public void zeroHeading() {
    zeppeli.reset();

    if (config == null) {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getCurrentSpeeds,
          (speeds, feedforwards) -> drive(speeds, false),
          new PPHolonomicDriveController(
              new PIDConstants(10, 0, 0),
              new PIDConstants(10, 0, 0)),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    }

  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {

      try {
        Thread.sleep(1000);
        zeroHeading();
        zeppeli.enableLogging(false);
      } catch (Exception e) {
      }
    }).start();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();

  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(zeppeli.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return Constants.RobotStructure.DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
        backLeft.getState(), backRight.getState());
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean doReset) {
    if (doReset) {
      zeroHeading();
    }
    setModuleStates(Constants.RobotStructure.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() };
  }

  StructArrayPublisher<SwerveModuleState> moduleState = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Module States", SwerveModuleState.struct).publish();

  @Override
  public void periodic() {
    field.setRobotPose(getPose());
    poseEstimator.update(zeppeli.getRotation2d(), getPositions());
    // SmartDashboard.putNumber("Robot Heading", getHeading());
    // // SmartDashboard.putString("Robot Location",
    // // getPose().getTranslation().toString());
    SmartDashboard.putData("Robot Location", field);
    moduleState.set(getModuleStates());

    // System.out.println(gyro.getAngle());

  }

  public void updateOdometry() {
    poseEstimator.update(
        zeppeli.getRotation2d(),
        getPositions());

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (Math.abs(zeppeli.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                           // vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.PHYSICAL_MAX_SPEED_MPS);
    // frontLeft.setDesiredState(desiredStates[2]);
    // frontRight.setDesiredState(desiredStates[3]);
    // backLeft.setDesiredState(desiredStates[0]);
    // backRight.setDesiredState(desiredStates[1]);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = { frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        backRight.getState() };
    return states;
  }

}
