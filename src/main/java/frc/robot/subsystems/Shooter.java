package frc.robot.subsystems;
import frc.robot.subsystems.Stuff;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotVersion2023;
import frc.robot.Constants.RobotVersionConstants;
import frc.robot.Other.RobotVersion;
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
public class Shooter extends SubsystemBase {
    public CANSparkMax AimingMotor = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax ShootingMotor1  = new CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax ShootingMotor2  = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    public RelativeEncoder AimingEncoder = AimingMotor.getEncoder();
    public RelativeEncoder ShootingEncoder1 = ShootingMotor1.getEncoder();
    public RelativeEncoder ShootingEncoder2 = ShootingMotor2.getEncoder();
    public Shooter() {

    }
    
    
    public Command RunShootingMotorAuto() {
        return run(
        () -> {
           ShootingMotor1.set(1); 
           ShootingMotor2.set(-1); 
        }
        );
    }
    public Command StopShootingMotorAuto(){
         return run(
        () -> {
           ShootingMotor1.set(0); 
           ShootingMotor2.set(0); 
        }
        );
    }
}