package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DriveConstants;

public class OdometryThread implements Runnable {

    private final Canandgyro m_imu = new Canandgyro(DriveConstants.kIMUCanID);

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    private SwerveModule[] m_swerveModules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

    private SwerveDriveOdometry m_odometry;

    public OdometryThread(){
      m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveId, 
        DriveConstants.kFrontLeftTurnId,
        DriveConstants.kFrontLeftTurnEncoderPort, 
        DriveConstants.kFrontLeftTurnEncoderOffset, 
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningReversed, 
        "FrontLeft"
      );
      m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveId, 
        DriveConstants.kFrontRightTurnId,
        DriveConstants.kFrontRightTurnEncoderPort, 
        DriveConstants.kFrontRightTurnEncoderOffset,
        DriveConstants.kFrontRightDriveReversed,
        DriveConstants.kFrontRightTurningReversed, 
        "FrontRight"
      );
      m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveId, 
        DriveConstants.kBackLeftTurnId,
        DriveConstants.kBackLeftTurnEncoderPort,
        DriveConstants.kBackLeftTurnEncoderOffset, 
        DriveConstants.kBackLeftDriveReversed,
        DriveConstants.kBackLeftTurningReversed, 
        "BackLeft"
      );
      m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveId, 
        DriveConstants.kBackRightTurnId,
        DriveConstants.kBackRightTurnEncoderPort, 
        DriveConstants.kBackRightTurnEncoderOffset, 
        DriveConstants.kBackRightDriveReversed, 
        DriveConstants.kBackRightTurningReversed, 
        "BackRight"
      );
      m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
    }

    public Rotation2d getAngle(){
      double angle = m_imu.getYaw();
      angle *= 360;
      return Rotation2d.fromDegrees(angle);
    }      

    public double getAngleDegrees() {
      return Math.IEEEremainder(-m_imu.getYaw() * 360, 360);
    }
      
    public void resetHeading() {
      m_imu.setYaw(0.0);
    }

    public void adjustAngle(double angle){
      m_imu.setYaw(angle);
    }

    public double getAngularVelocityYaw() {
      return m_imu.getAngularVelocityYaw();
    }

    public void resetOdoToPose(double xStartPose, double yStartPose){
      m_odometry.resetPosition(getAngle(), new SwerveModulePosition[] {
          m_frontLeft.getPosition(), m_frontRight.getPosition(),
          m_backLeft.getPosition(), m_backRight.getPosition()
        }, new Pose2d(xStartPose, yStartPose, getAngle()));
    }

    public void resetOdo(Pose2d pose) {
      m_odometry.resetPosition(getAngle(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }, pose);
    }

    public Pose2d getPoseMeters(){
      return m_odometry.getPoseMeters();
    }

    public synchronized void setModuleStates(SwerveModuleState[] states) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTranslationalMetersPerSecond);

      m_frontLeft.setDesiredState(states[0]);
      m_frontRight.setDesiredState(states[1]);
      m_backLeft.setDesiredState(states[2]);
      m_backRight.setDesiredState(states[3]);
    }

    public synchronized SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++){
        states[i] = m_swerveModules[i].getState();
      }
      return states;
    }

    private synchronized SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        positions[i] = m_swerveModules[i].getPosition();
      } 
      return positions;
    }


    @Override
    public void run(){
      m_odometry.update(getAngle(), getModulePositions());
    }

}
