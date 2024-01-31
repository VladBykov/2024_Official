// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Represents a swerve drive style drivetrain. */

public class DriveTrainPID extends SubsystemBase {
  public PIDController m_xController = new PIDController(1, 0, 0);
  public PIDController m_yController = new PIDController(1, 0, 0);
  public ProfiledPIDController m_thetaController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kMaxAngularSpeed, kModuleMaxAngularAcceleration));

  public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(DriveTrainPID.m_frontLeftLocation,
      DriveTrainPID.m_frontRightLocation, DriveTrainPID.m_backLeftLocation, DriveTrainPID.m_backRightLocation);

  public boolean m_WheelLock = false;
  public boolean m_FieldRelativeEnable = true;
  public static final double kMaxSpeed = 3.68; // WP this seemed to work don't know why // 3.68 meters per second or
                                               // 12.1
  // ft/s (max speed of SDS Mk3 with Neo motor)
  public static final double kMaxAngularSpeed = Math.PI / 3; // 1/2 rotation per second
  public static final double kModuleMaxAngularAcceleration = Math.PI / 3;
  private final AHRS navx = new AHRS();

  public final static Translation2d m_frontRightLocation = new Translation2d(0.285, -0.285);
  public final static Translation2d m_frontLeftLocation = new Translation2d(0.285, 0.285);
  public final static Translation2d m_backLeftLocation = new Translation2d(-0.285, 0.285);
  public final static Translation2d m_backRightLocation = new Translation2d(-0.285, -0.285);

  // constructor for each swerve module
  public final SwerveModule m_frontRight = new SwerveModule(Constants.frDriveMotorChannel,
      Constants.frSteerMotorChannel, Constants.frEncoderChannel, .777 );
  public final SwerveModule m_frontLeft = new SwerveModule(Constants.flDriveMotorChannel, Constants.flSteerMotorChannel,
      Constants.flEncoderChannel, .6168);
  public final SwerveModule m_backLeft = new SwerveModule(Constants.blDriveMotorChannel, Constants.blSteerMotorChannel,
      Constants.blEncoderChannel, .519 );
  public final SwerveModule m_backRight = new SwerveModule(Constants.brDriveMotorChannel, Constants.brSteerMotorChannel,
      Constants.brEncoderChannel, .625); // 0.05178
      //0.9262



  // INITIAL POSITIONS to help define swerve drive odometry. THis was a headache
  public SwerveDriveKinematics m_initialStates;

  private final SwerveDriveOdometry m_odometry;

  public boolean flipFieldPose() {
    // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  // Set up custom logging to add the current path to a field 2d widget
  /*
   * PathPlannerLogging.setLogActivePathCallback((poses) ->
   * field.getObject("path").setPoses(poses));
   * () -> {
   * // Boolean supplier that controls when the path will be mirrored for the red
   * alliance
   * // This will flip the path being followed to the red side of the field.
   * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   * 
   * var alliance = DriverStation.getAlliance();
   * if (alliance.isPresent()) {
   * return alliance.get() == DriverStation.Alliance.Red;
   * }
   * return false;
   * },
   * this
   * );
   */

  // Set up custom logging to add the current path to a field 2d widget
  // PathPlannerLogging.setLogActivePathCallback((poses) ->
  // field.getObject("path").setPoses(poses));
  public Pose2d getPose2d() {
    Pose2d current_pose_meters = m_odometry.getPoseMeters();
    
    return current_pose_meters;
  }

  // Constructor
  public DriveTrainPID() {
    AutoBuilder.configureHolonomic(
        this::getPose2d,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveChassisSpeeds,
        Constants.pathFollowerConfig,
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
    m_initialStates = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation);

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        navx.getRotation2d(),
        getSwerveModulePositions());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontLeftPosition = m_frontLeft.getModulePosition();
    SwerveModulePosition frontRightPosition = m_frontRight.getModulePosition();
    SwerveModulePosition backLeftPosition = m_backLeft.getModulePosition();
    SwerveModulePosition backRightPosition = m_backRight.getModulePosition();

    return new SwerveModulePosition[] {
        frontLeftPosition,
        frontRightPosition,
        backLeftPosition,
        backRightPosition
    };
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot) {

    SmartDashboard.putNumber("Robot/Command/X Speed", xSpeed);
    SmartDashboard.putNumber("Robot/Command/Y Speed", ySpeed);
    SmartDashboard.putBoolean("Robot/Field Oriented?", m_FieldRelativeEnable);

    Rotation2d robotRotation = new Rotation2d(navx.getRotation2d().getRadians());

    // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        m_FieldRelativeEnable ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    if (!m_WheelLock) {
      setModuleStates(swerveModuleStates);
    } else {
      WheelLock();
    }

  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.kDriveMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void WheelLock() {
    m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
    m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
    m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
    m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
  }

  @Override
  public void periodic() {
    updateOdometry();
    Pose2d currentPose = this.getPose2d();
    ChassisSpeeds currentChassisSpeeds = this.getChassisSpeeds();
    SmartDashboard.putNumber("Robot/Odometry/Pose X", currentPose.getX());
    SmartDashboard.putNumber("Robot/Odometry/Pose Y", currentPose.getY());
    SmartDashboard.putNumber("Robot/Odometry/Pose Rot", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds X", currentChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds Y", currentChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot/Odometry/Chassis Speeds Rot", Units.radiansToDegrees(currentChassisSpeeds.omegaRadiansPerSecond));

    SmartDashboard.putNumber("Robot/Odometry/navx/Rotation", navx.getRotation2d().getDegrees());
    
    super.periodic();
  }

  public Command WheelzLock() {

    return runOnce(
        () -> {

          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (m_WheelLock == true) {
            m_WheelLock = false;
          } else if (m_WheelLock == false) {
            m_WheelLock = true;
          }
        });
  }

  public Command ZeroGyro() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          navx.reset();

        });
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeed) {
    drive(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, chassisSpeed.omegaRadiansPerSecond);
  }

  public void resetPose(Pose2d pose2d) {
    resetOdometry(pose2d);

  }

  public void resetOdometry(Pose2d pose2d) {
    m_odometry.resetPosition(navx.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  /**
   * 
   * Converts raw module states into chassis speeds
   * 
   * @return chassis speeds object
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getModuleState(),
        m_frontRight.getModuleState(),
        m_backLeft.getModuleState(),
        m_backRight.getModuleState()
    };
  }

  public Command resetPose2d() {
    return runOnce(
        () -> {
          resetPose(new Pose2d());
        });
  }

  public Command toggleFieldRelativeEnable() {

    return runOnce(
        () -> {
          // System.out.println("I am Here");
          // one-time action goes here
          // WP - Add code here to toggle the gripper solenoid
          if (m_FieldRelativeEnable == true) {
            m_FieldRelativeEnable = false;
            // System.out.println("I am Here 2");
          } else if (m_FieldRelativeEnable == false) {
            m_FieldRelativeEnable = true;
            // System.out.println("I am Here 3");
          }
        });
  }


  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        getSwerveModulePositions());
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }
}
