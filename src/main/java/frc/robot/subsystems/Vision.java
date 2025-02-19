// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera m_cam = new PhotonCamera("Arducam_OV9281_USB_Camera"); //TODO: Change to actual camera name
  private Transform3d m_robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //TODO: Change this later!
  
  private List<PhotonPipelineResult> m_result;
  private boolean m_hasTargets;
  private List<PhotonTrackedTarget> m_targets;
  private PhotonTrackedTarget m_bestTarget;

  private double m_yaw;
  private double m_pitch;
  private double m_area;
  private double m_skew;

  private double m_targetPitch;
  private double m_targetYaw;
  private boolean m_targetInView;
  private int m_targetID = 0;

  private boolean m_tag6InView;
  private boolean m_tag7InView;
  private boolean m_tag8InView;
  private boolean m_tag9InView;
  private boolean m_tag10InView;
  private boolean m_tag11InView;
  private boolean m_tag17InView;
  private boolean m_tag18InView;
  private boolean m_tag19InView;
  private boolean m_tag20InView;
  private boolean m_tag21InView;
  private boolean m_tag22InView;

  private Transform3d m_camToTarget;
  private List<TargetCorner> m_corners;
  private int m_ID;
  private double m_poseAmbiguity;

  private AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Pose3d m_robotPose;
  private PhotonPoseEstimator m_poseEstimator;
  // Creates a new Vision. 
  public Vision() {
    
  }

  public PhotonCamera getCam(){
    return m_cam;
  }

  public double getYaw(){
    return m_yaw;
  }

  public double getPitch(){
    return m_pitch;
  }

  public double getArea(){
    return m_area;
  }

  public double getSkew(){
    return m_skew;
  }

  public Transform3d getCameraToTarget(){
    return m_camToTarget;
  }

  public List<TargetCorner> getCorners(){
    return m_corners;
  }

  public int getID(){
    return m_ID;
  }

  public double getPoseAmbiguity(){
    return m_poseAmbiguity;
  }

  public double getTargetYaw(){
    return m_targetYaw;
  }

  public double getTargetPitch(){
    return m_targetPitch;
  }

  public boolean getTargetInView(){
    return m_targetInView;
  }

  public void setTargetID(int targetID){
    m_targetID = targetID;
  }

  public int getTargetID(){
    return m_targetID;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    Optional<EstimatedRobotPose> m_visionEst = Optional.empty();
    for (var change : m_cam.getAllUnreadResults()) {
        m_visionEst = m_poseEstimator.update(change);
    }
    return m_visionEst;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_result = m_cam.getAllUnreadResults();
    if (!m_result.isEmpty()) {
      m_targets = m_result.get(0).getTargets().get();
      m_bestTarget = m_result.getBestTarget();

      for(PhotonTrackedTarget target : m_result){
        if (m_targetID != 0 && target.getFiducialId() == m_targetID){
          m_targetInView = true;
          m_targetYaw = target.getYaw();
          m_targetPitch = target.getPitch();
        }
      }
    }

    if(m_bestTarget != null){
      //m_vision = m_bestTarget.getCam();
       m_yaw = m_bestTarget.getYaw();
       m_pitch = m_bestTarget.getPitch();
       m_area = m_bestTarget.getArea();
       m_skew = m_bestTarget.getSkew();
       m_camToTarget = m_bestTarget.getBestCameraToTarget();
       m_corners = m_bestTarget.getDetectedCorners();
       m_ID = m_bestTarget.getFiducialId();
       m_poseAmbiguity = m_bestTarget.getPoseAmbiguity();
       m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(m_camToTarget, m_aprilTagFieldLayout.getTagPose(m_ID).get(), m_robotToCam);
       m_poseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);
    }
   
  }
}
*/