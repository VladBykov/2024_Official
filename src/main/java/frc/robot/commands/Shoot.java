package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
public class Shoot extends Command {
  // regular shooting fileS
private final Shooter m_ShooterSubSystem;
private final Supplier<AprilTag> m_aprilTagProvider;
    public Shoot(Shooter shooterSub, Supplier<AprilTag> aprilTagSupplier){
    m_ShooterSubSystem = shooterSub;
    m_aprilTagProvider = aprilTagSupplier;
    addRequirements(m_ShooterSubSystem);
    }
    public void execute(){
        AprilTag aprilTag  = m_aprilTagProvider.get();
        if (aprilTag.ID <= 0) { // is valid if > 0: we update our current estimate of where the april tag is relative to the robot
            m_ShooterSubSystem.StopShootingMotorAuto();
          return;
        }
        Pose3d RobotPoseToLimelight = aprilTag.pose;
        
        if (RobotPoseToLimelight.getZ() >= 1.95 || RobotPoseToLimelight.getZ() <= 2.05) {
            // set to desired angle/position (pre-set)
        }
        else if (RobotPoseToLimelight.getZ() >= 2.05) {
            // shooter Shoots faster depending on its pose, Using PID 
          }
        else if (RobotPoseToLimelight.getZ() <= 1.95) {
            // shooter shoots slower depending on its pose, Using PID
          }
    }
}
