// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {


    public static class SwerveModule{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        public static final double kDriveMotorGearRatio = 1/6.746031746031747;
        public static final double kDriveEncoderRotation2Meter = (kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI); 
        public static final double kDriveEncoderRPS2MPS = kDriveEncoderRotation2Meter;

        public static final double kTurnMotorGearRatio = 1/12.8;
        public static final double kTurnEncoderRotation2Rad = (kTurnMotorGearRatio * 2 * Math.PI);   
        public static final double kTurnEncoderRPS2MPS = kTurnEncoderRotation2Rad;
        
        public static final double kPturn = 0.118;
        public static final double kDturn= 0;
    }

    public static class DriveConstants{
        public static final double kPhysicalMaxSpeedMPS = 18;
        public static final double kPhysicalMaxAngularSpeedRPM = 30 * Math.PI;


        public static final double kTeleDriveMaxAccelerationUPS = 5;
        public static final double kTeleDriveMaxAngularAccelerationUPS = 5;

        public static final double kTeleDriveMaxSpeedMPS = kPhysicalMaxSpeedMPS/ 2;
        public static final double kTeleDriveMaxAngularSpeedRPS = kPhysicalMaxAngularSpeedRPM / 2;
        
        public static final int kXAxis = 0;
        public static final int kYAxis = 1;
        public static final int kTurnAxis = 4;
        public static final int kFieldOrientedButton = 0;

        public static final double kJoystickDeadband = 0.05;
    }

    public static class RobotStructure{

        public static final double kTrackWidth = Units.inchesToMeters(24.75);
        public static final double kWheelBase = Units.inchesToMeters(19);

        public static final Translation2d frontLeftLocation = new Translation2d(kWheelBase/2, -kTrackWidth/2);
        public static final Translation2d frontRightLocation = new Translation2d(kWheelBase/2, kTrackWidth/2);
        public static final Translation2d BackLeftLocation = new Translation2d(-kWheelBase/2, -kTrackWidth/2);
        public static final Translation2d BackRightLocation =    new Translation2d(-kWheelBase/2, kTrackWidth/2);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            BackLeftLocation,
            BackRightLocation
        );
        
    }

    public static class FrontLeft{

        public static final int kDriveMotorID = 11;
        public static final int kTurnMotorID = 12;
        public static final boolean kDriveMotorReversed = false;
        public static final boolean kTurningMotorReversed = true;
        public static final int kAbsoluteEncoderID = 1;
        public static final double kFrontLeftEncoderOffset = 0.227783203125;
    }

    public static class FrontRight{
        
        public static final int kDriveMotorID = 41;
        public static final int kTurnMotorID = 42;
        public static final boolean kDriveMotorReversed = true;
        public static final boolean kTurningMotorReversed = true;
        public static final int kAbsoluteEncoderID = 4;
        public static final double kFrontRightEncoderOffset = 0.248291015625;
    }

    public static class BackLeft{
        
        public static final int kDriveMotorID = 21;
        public static final int kTurnMotorID = 22;
        public static final boolean kDriveMotorReversed = true;
        public static final boolean kTurningMotorReversed = true;
        public static final int kAbsoluteEncoderID = 2;
        public static final double kBackLeftEncoderOffset = -0.29052734375;

    }

    public static class BackRight{
        
        public static final int kDriveMotorID = 31;
        public static final int kTurnMotorID = 32;
        public static final boolean kDriveMotorReversed = false;
        public static final boolean kTurningMotorReversed = true;
        public static final int kAbsoluteEncoderID = 3;
        public static final double kBackRightEncoderOffset = 0.188232421875;
    }

    public static class Hook{
        // put motor id in lefthook
        public static final int leftHook = 61;
        public static final int rightHook = 62;
        public static final double upSpeed = -0.17;
        public static final double downSpeed = 0.5;
        public static final double min = 0.001;
        public static final double max = -4.26;
    }

    public static class Intake{
        public static final int m_1ID = 51;
        public static final int m_2ID = 52;
        public static final boolean m_1Inverted = true;
        public static final boolean m_2Inverted = true;
        public static final double speed = 0.05;
    }

    public static class Shooter{
        public static final int mID = 8;
        public static final boolean mInverted = false;
        public static final double launchSpeed = 0.69420;
    }


    public static class PathPlannerConstants{
        // public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
        //     new PIDConstants(5,0,0),
        //     new PIDConstants(5,0,0),
        //     DriveConstants.kPhysicalMaxSpeedMPS,
        //     0, //max distance to wheel
        //     new ReplanningConfig());    
    }

    public static class Photon{
        public static final int blueIDP = 7;
        public static final int blueIDS = 8;

        public static final int redIDS = 3;
        public static final int redIDP = 4;

        // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0,0.5), 
            new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        
            /*

        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d pose = target.getBestCameraToTarget();
        List<org.photonvision.targeting.TargetCorner> corners = target.getDetectedCorners();


        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();

        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), 
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
            new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0)));

        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
        
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {

            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

            return photonPoseEstimator.update();
        }

        public void Snapshot () {
            // Capture pre-process camera stream image
            camera.takeInputSnapshot();
            // Capture post-process camera stream image
            camera.takeOutputSnapshot();

        }
        */
    }

}
