package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;

public class OdometryThread implements Runnable {

    private final Canandgyro m_imu = new Canandgyro(DriveConstants.kIMUCanID);

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    private SwerveModule[] m_swerveModules = new SwerveModule[4]; 

    private SwerveDriveOdometry m_odometry;

    public OdometryThread(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight){
      m_frontLeft = frontLeft;
      m_frontRight = frontRight;
      m_backLeft = backLeft;
      m_backRight = backRight;

      m_swerveModules[0] = m_frontLeft;
      m_swerveModules[1] = m_frontRight;
      m_swerveModules[2] = m_backLeft;
      m_swerveModules[3] = m_backRight;

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
      System.out.println("Running odometry thread");
      SmartDashboard.putNumber("Angle", getAngleDegrees());
      SmartDashboard.putNumber("Pose X", getPoseMeters().getX());
      SmartDashboard.putNumber("Pose Y", getPoseMeters().getY());
    }

}
