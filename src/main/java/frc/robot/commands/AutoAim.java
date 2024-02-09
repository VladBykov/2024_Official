package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPID;
import frc.robot.subsystems.Shooter;
public class AutoAim extends Command {
    private final Supplier<AprilTag> m_aprilTagProvider;
    private final Shooter m_ShooterSub;

    public AutoAim(Supplier<AprilTag> aprilTagSupplier, Shooter ShooterSub) {
     m_aprilTagProvider = aprilTagSupplier;
     m_ShooterSub = ShooterSub;
     addRequirements(m_ShooterSub);
    }
    public void execute (){
    
    AprilTag m_aprilTag = m_aprilTagProvider.get();
    Pose3d botToTargetPose = m_aprilTag.pose;
    if (m_aprilTag.ID <= 0) { // is valid if > 0: we update our current estimate of where the april tag is relative to the robot
        m_ShooterSub.AimingMotor.set(0);
        return;
      }
      
      Pose2d targetPose = new Pose2d(
      botToTargetPose.getTranslation().toTranslation2d(),
      Rotation2d.fromRadians(Math.atan2(botToTargetPose.getY(), botToTargetPose.getX())));
    if (botToTargetPose.getZ() >= 1.95 && botToTargetPose.getZ() <= 2.05){
    // Shooter Rotates to desired angle/ position
      }
    else if (botToTargetPose.getZ() <= 1.95) {
        // shooter corrects by moving up depending on its pose, Using PID
      }
    else if (botToTargetPose.getZ() >= 2) {
        // shooter corrects by moving down depending on its pose, Using PID
      }


    
/*  NEED TO DO:
        
    if botToTargetPose == ideal spot (whatever TagToGoal is)
        Set perfect angle
    else 
        Do PID so that it is the ideal angle (Hits where our ideal spot is)
      ROUGH IDEA
*/
    }
}
