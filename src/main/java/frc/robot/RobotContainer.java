
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrainPID;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ManualRotateArmCommand;
import frc.robot.commands.AutoRotateArmCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Tags;
import frc.robot.subsystems.Stuff;
import frc.robot.subsystems.SwerveModule;
//add in later
//import frc.robot.commands.AprilAlignCommand;

public class RobotContainer {
  DriveTrainPID m_DriveTrainPID = new DriveTrainPID();
  Arm m_Arm = new Arm();
  Stuff m_Stuff = new Stuff();
  Tags m_Tags = new Tags();

  XboxController driverController = new XboxController(Constants.DriverControllerChannel);
  XboxController manipController = new XboxController(Constants.ManipControllerChannel);
  JoystickButton driverButtonB = new JoystickButton(driverController, Constants.buttonB);
  JoystickButton manipButtonA = new JoystickButton(manipController, Constants.buttonA);
  JoystickButton driverButtonA = new JoystickButton(driverController, Constants.buttonA);

  JoystickButton driverButtonRight = new JoystickButton(driverController, Constants.buttonRight);
  JoystickButton driverButtonLeft = new JoystickButton(driverController, Constants.buttonLeft);
  JoystickButton driverButtonOption = new JoystickButton(driverController, Constants.buttonOptions);
  // Constants.buttonX);
  JoystickButton driverButtonX = new JoystickButton(driverController, Constants.buttonX);
  JoystickButton driverButtonRS = new JoystickButton(driverController, Constants.buttonRS);
  JoystickButton driverButtonLS = new JoystickButton(driverController, Constants.buttonLS);
  JoystickButton manipButtonB = new JoystickButton(manipController, Constants.buttonB);
  JoystickButton manipButtonY = new JoystickButton(manipController, Constants.buttonY);
  JoystickButton manipButtonRight = new JoystickButton(manipController, Constants.buttonRight);
  JoystickButton manipButtonLeft = new JoystickButton(manipController, Constants.buttonLeft);
  JoystickButton manipButtonOptions = new JoystickButton(manipController, Constants.buttonOptions);
  JoystickButton driverButtonOptions = new JoystickButton(driverController, Constants.buttonOptions);
  JoystickButton manipButtonRS = new JoystickButton(manipController, Constants.buttonRS);
  // A chooser for autonomous commands
  SendableChooser<Integer> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureShuffleboard();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->comand mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /// new Trigger(m_exampleSubsystem::exampleCondition)
    /// .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    /// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_DriveTrainPID.setDefaultCommand(new SwerveDriveCommand(() -> driverController.getLeftY(),
        () -> driverController.getLeftX(), () -> driverController.getRightX(), m_DriveTrainPID));
    // limelight allign works on both controllers
    // manipButtonX.whileTrue(new AlignCommand(m_DriveTrain, () ->
    // frc.robot.subsystems.Stuff.angle));
    driverButtonX.whileTrue(new AlignCommand(m_DriveTrainPID, () -> frc.robot.subsystems.Stuff.angle));
    // manipButtonB.whileTrue(new AprilAlignCommand(m_DriveTrain, () ->
    // frc.robot.subsystems.Tags.tx2));
    // driverButtonB.whileTrue(new FieldAlignedCommand(m_DriveTrain));
    driverButtonRS.onTrue(m_DriveTrainPID.WheelzLock());
    driverButtonB.onTrue(m_DriveTrainPID.ZeroGyro());
    driverButtonA.onTrue(m_DriveTrainPID.toggleFieldRelativeEnable());
    // WP - DO NOT UNCOMMENT WITHOUT TALKING TO WARD
    driverButtonOptions.onTrue(m_DriveTrainPID.resetPose2d());
    m_Arm.setDefaultCommand(new AutoRotateArmCommand(m_Arm));
    manipButtonLeft.onTrue(m_Arm.LowerArm()); // starts at 1 (5 deegrees) goes down
    manipButtonRight.onTrue(m_Arm.RaiseArm());
    driverButtonOption.onTrue(m_DriveTrainPID.resetPose2d()); // starts at 1, when pressed goes up to 2 (82 Deegrees),
                                                              // when pressed
    // again goes up to 3 (85 deegrees)
    // TODO RT Accelerate LT Deaccelerate

  }

  private void configureShuffleboard() {
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Auto 1", 1);
    m_chooser.addOption("Auto 2", 2);
    m_chooser.addOption("Auto 3", 3);

    SmartDashboard.putData(m_chooser);

    // SmartDashboard.putData(m_DriveTrainPID.GetPose2d().getTranslation());

  }

  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig
    (12.1, 8).setKinematics(Constants.m_kinematics); // we don't know our acceleration 
   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), List.of(new Translation2d(1,0), new Translation2d(1,-1)), new Pose2d(2, -1, Rotation2d.fromDegrees(180)),trajectoryConfig);
  
  
  PIDController xController = new PIDController(1.5, 0, 0);
  PIDController yController = new PIDController(1.5, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(3, 0,0, Constants.kthetaController);

  SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
trajectory, m_DriveTrainPID::GetPose2d, Constants.m_kinematics, xController, yController, thetaController, m_DriveTrainPID::setModuleStates, m_DriveTrainPID);

return new SequentialCommandGroup(new InstantCommand(() -> m_DriveTrainPID.resetOdometry(trajectory.getInitialPose())), new InstantCommand(() -> m_DriveTrainPID.stopModules()));

/*    Command autoSeq = Commands.sequence(
        m_DriveTrainPID.ZeroGyro(),
        Commands.waitSeconds(1.0),
        new AutoCommand(m_DriveTrainPID, m_chooser.getSelected()));
    return autoSeq;
    // return new AutoCommand(m_DriveTrain);*/
  }
}