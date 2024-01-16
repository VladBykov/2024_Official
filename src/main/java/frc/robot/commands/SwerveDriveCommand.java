
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveAndDriveConstants;

public class SwerveDriveCommand extends Command {
  DoubleSupplier m_leftY;
  DoubleSupplier m_leftX;
  DoubleSupplier m_rightX;
  // double leftY;
  // double leftX;
  // double rightX;
  DriveTrain m_DriveTrain;
  RobotContainer RobotContainer;
  // double x;
  // double y;
  // double rot;
  // double w1ca;
  // double w2ca;
  // double w3ca;
  // double w4ca;
  /**
   * Creates a constructor for our SwerveDriveCommand<br>
   * <br>
   * This tells the robot to drive
   * @param _leftY
   * @param _leftX
   * @param _rightX
   * @param m_DriveTrain
   */
  public SwerveDriveCommand(DoubleSupplier _leftY, DoubleSupplier _leftX, DoubleSupplier _rightX,
    DriveTrain _DriveTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _DriveTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * m_DriveTrain.brDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.frDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.blDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     * m_DriveTrain.flDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Double x;
    Double y;
    Double rotation;
    Double leftX = m_leftX.getAsDouble();
    Double leftY = m_leftY.getAsDouble();
    Double rightX = m_rightX.getAsDouble();
    // System.out.println();

    // Finds the X Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(leftX) < MiscConstants.deadzone) {
      x = 0.0;
    } else {
      x = -leftX;
    }

    // Finds the Y Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(leftY) < MiscConstants.deadzone) {
      y = 0.0;
    } else {
      y = -leftY;
    }

    // Finds the X Value of the Right Stick on the Controller and Takes Care of
    // Joystick Drift
    if (Math.abs(rightX) < MiscConstants.deadzone) {
      rotation = 0.0;
    } else {
      rotation = -rightX;
    }

    // Swerve drive uses a different Y and X than expected!

    m_DriveTrain.drive(y, x, rotation, SwerveAndDriveConstants.FieldRelativeEnable, SwerveAndDriveConstants.WheelLock);
    Pose2d pose = m_DriveTrain.GetPose2d();
    System.out.println(pose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
