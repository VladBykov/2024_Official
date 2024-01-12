package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPID;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class AutonomousPathCommand extends Command {  
  private Field2d field = new Field2d();

  private DriveTrainPID m_driveTrain;


  public AutonomousPathCommand(
    DriveTrainPID driveTrain
  ){
    m_driveTrain = driveTrain;
  }
  
//   public SwerveSubsystem() {
//     gyro = new SimGyro();
//     modules = new SimSwerveModule[]{
//       new SimSwerveModule(),
//       new SimSwerveModule(),
//       new SimSwerveModule(),
//       new SimSwerveModule()
//     };
//     kinematics = new SwerveDriveKinematics(
//       Constants.Swerve.flModuleOffset, 
//       Constants.Swerve.frModuleOffset, 
//       Constants.Swerve.blModuleOffset, 
//       Constants.Swerve.brModuleOffset
//     );
//     odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

//     // Configure AutoBuilder
//     AutoBuilder.configureHolonomic(
//       this::getPose, 
//       this::resetPose, 
//       this::getSpeeds, 
//       this::driveRobotRelative, 
//       Constants.Swerve.pathFollowerConfig,
//       () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//           var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//               return alliance.get() == DriverStation.Alliance.Red;
//           }
//           return false;
//       },
//       this
//     );

//     // Set up custom logging to add the current path to a field 2d widget
//     PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

//     SmartDashboard.putData("Field", field);
//   }

}
