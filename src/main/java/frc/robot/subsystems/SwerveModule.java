// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType; //TODO Was giving us weird error 
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
/**
 * This is the code to run a single swerve module <br>
 * <br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
  
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

   // private static final double rpstoPositionScaler = (Constants.kWheelCircumference * Constants.driveEncoderCtsperRev)
    //        / (2 * Math.PI);// (Constants.kWheelDiameterM * Constants.NeoEncoderCountsPerRev) /
                            // (Constants.GearRatio * (Math.PI * 2));
    //private static final double rpmToVelocityScaler = 3 * (Constants.kWheelCircumference / Constants.GearRatio) / 60; // SDS
                                                                                                                      // Mk3
                                                                                     // standard
    private static final double rpmToVelocityScaler = (Constants.kWheelDiameterM*Math.PI) / Constants.GearRatio / 60;
    XboxController m_xboxController = new XboxController(0);

    public static final double revMaxMotorMaxRPM = 5676.0;
    public static final double revMaxMotorMaxSpeed = revMaxMotorMaxRPM * rpmToVelocityScaler;
    
    // gear
                                                                                                     // GOES
                                                                                                                      // WRONG
                                                                                                                      // TODO
    // from motor to wheel, divide
    // by 60 to go from secs to
    // mins
    // kWheelCircumference used to be
    public static final double kModuleMaxAngularVelocity = DriveTrainPID.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public final CANSparkMax m_driveMotor;
    public final CANSparkMax m_turningMotor;

    private final SparkPIDController m_drivePID;
  
    public final RelativeEncoder m_driveEncoder;
    private final DutyCycleEncoder m_turningEncoder;
    private final DigitalInput m_TurnEncoderInput;
    public final DutyCycle m_TurnPWMEncoder;
    private double turnEncoderOffset;
    private double encoderBias = 0; // encoder stuff for rotation
    private int turnPWMChannel;

    // pid stuff added 1-18-24
    public double m_NeoMaxRPM = Constants.NeoMaxRpm;
    private double kP = 0.3;
    private double kI = 0;
    private double kD = 0;
    private double kIz = 0;
    private double kFF = 0.000015;
    private double kMaxOutput = 1;
    private double kMinOutput = -kMaxOutput;
    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel     CAN ID for the drive motor.
     * @param turningMotorChannel   CAN ID for the turning motor.
     * @param driveEncoder          DIO input for the drive encoder channel A
     * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
     * @param turnOffset            offset from 0 to 1 for the home position of the
     *                              encoder
     */

    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turnEncoderPWMChannel, double turnOffset) {
        // can spark max motor controller objects
        
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        m_driveMotor = new CANSparkMax(driveMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        //m_driveMotor.setOpenLoopRampRate(0.1);
        m_driveMotor.restoreFactoryDefaults();
        m_drivePID = m_driveMotor.getPIDController();

        // 1-18-24 added for pid
        m_drivePID.setP(kP);
        m_drivePID.setI(kI);
        m_drivePID.setD(kD);
        m_drivePID.setIZone(kIz);
        m_drivePID.setFF(kFF);
        m_drivePID.setOutputRange(kMinOutput, kMaxOutput);

        // m_drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_drivePID.setSmartMotionMaxAccel(0.2, 0);
       // m_drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);

        // if coefficient changes

        // m_turningEncoder.setPositionConversionFactor(1/4096);

        // spark max built-in encoder
        m_driveEncoder = m_driveMotor.getEncoder();
        // m_driveEncoder.setPositionConversionFactor(rpstoPositionScaler);

       m_driveEncoder.setVelocityConversionFactor(rpmToVelocityScaler);
       SmartDashboard.putNumber("RPM Conversion Factor", rpmToVelocityScaler);
        // m_driveEncoder.setPositionConversionFactor((1/4096)/(8.14)); // encoder rev
        // per rotation / gear ratio
        // limit power to motors 3/25/23
        // m_driveMotor.setSmartCurrentLimit(30, 40);
        // m_turningMotor.setSmartCurrentLimit(30, 40);
        // m_driveMotor.burnFlash();
        // m_turningMotor.burnFlash();

        // TODO Im changing this
        // PWM encoder from CTRE mag encoders
        turnPWMChannel = turnEncoderPWMChannel;
        m_TurnEncoderInput = new DigitalInput(turnEncoderPWMChannel);
        m_TurnPWMEncoder = new DutyCycle(m_TurnEncoderInput);
        turnEncoderOffset = turnOffset;
        m_turningEncoder = new DutyCycleEncoder(m_TurnPWMEncoder);
        //m_turningEncoder.setDistancePerRotation((2 * Math.PI) /Constants.MagEncoderCountsPerRev);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        encoderBias = m_driveEncoder.getPosition();

        m_driveEncoder.setPosition(0);
        m_turningEncoder.reset();
        m_turningPIDController.reset(0);
    }

    public void periodic() {

        double p = SmartDashboard.getNumber("P Gain", kP);
        double i = SmartDashboard.getNumber("I Gain", kI);
        double d = SmartDashboard.getNumber("D Gain", kI);
        double iz = SmartDashboard.getNumber("I Zone", kIz);
        double ff = SmartDashboard.getNumber("Feed Forward", kFF);
        double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
        double min = SmartDashboard.getNumber("Min Output", kMinOutput);
        // m_turningEncoder.getCountsPerRevolution();
        super.periodic();

        SmartDashboard.putNumber("Drive Motor Speed (Mot.) Device:" + m_driveMotor.getDeviceId(), m_driveMotor.getAppliedOutput());
        SmartDashboard.putNumber("Drive Motor Speed (Enc.) Device:" + m_driveMotor.getDeviceId(), m_driveEncoder.getVelocity());

        // if change of PID coefficient

         if((p != kP)) { m_drivePID.setP(p); kP = p; }
         if((i != kI)) { m_drivePID.setI(i); kI = i; }
         if((d != kD)) { m_drivePID.setD(d); kD = d; }
         if((iz != kIz)) { m_drivePID.setIZone(iz); kIz = iz; }
         if((ff != kFF)) { m_drivePID.setFF(ff); kFF = ff; }
         if((max != kMaxOutput) || (min != kMinOutput)) {
            m_drivePID.setOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
         }
        }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveEncoder.getPosition(), new Rotation2d(getTurnEncoderRadians()));
    }

    /**
     * Returns the current velocity state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        // the getVelocity() function normally returns RPM but is scaled in the
        // SwerveModule constructor to return actual wheel speed
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderRadians()));
    }

    /**
     * 
     * @return Position of the drive motor and the orientation of the turning motor
     */
    public SwerveModulePosition getModulePosition() { // TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
        return new SwerveModulePosition((m_driveEncoder.getPosition() - encoderBias) * rpmToVelocityScaler * 60,
                new Rotation2d(getTurnEncoderRadians()));
   }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
        
       
       
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(0));
        
        
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(getTurnEncoderRadians()));
        SmartDashboard.putNumber("Set Desired State Swerve Module Dev." + m_driveMotor.getDeviceId(), optimizedState.speedMetersPerSecond);
        
       
        final double signedAngleDifference = closestAngleCalculator(getTurnEncoderRadians(), state.angle.getRadians());
        double rotateMotorPercentPower = signedAngleDifference / (2 * Math.PI); // proportion error control //2	        double maxrpm = 5767; 
        m_drivePID.setReference(optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);







        final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

        final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);


        m_turningMotor.setVoltage(turnOutput + turnFeedforward);










        //m_turningMotor.set(turnOutput);

        // m_driveMotor.set(state.speedMetersPerSecond);
        //m_drivePID.setReference(setpointX, CANSparkBase.ControlType.kVelocity);

    

    }

    /**
     * Applies the absolute encoder offset value and converts range
     * from 0-1 to 0-2 pi radians
     * 
     * @return Angle of the absolute encoder in radians
     */
    private double getTurnEncoderRadians() {
        double appliedOffset = (m_TurnPWMEncoder.getOutput() - turnEncoderOffset) % 1;
        SmartDashboard.putNumber("PWMChannel " + Integer.toString(turnPWMChannel), m_TurnPWMEncoder.getOutput());
        return appliedOffset * 2 * Math.PI;
    }

    /**
     * Calculates the closest angle and direction between two points on a circle.
     * 
     * @param currentAngle
     *                     <ul>
     *                     <li>where you currently are
     *                     </ul>
     *                     </li>
     * @param desiredAngle
     *                     <ul>
     *                     <li>where you want to end up
     *                     </ul>
     *                     </li>
     * @return
     *         <ul>
     *         <li>signed double of the angle (rad) between the two points
     *         </ul>
     *         </li>
     */
    public double closestAngleCalculator(double currentAngle, double desiredAngle) {
        double signedDiff = 0.0;
        double rawDiff = currentAngle > desiredAngle ? currentAngle - desiredAngle : desiredAngle - currentAngle; // find
                                                                                                                  // the
                                                                                                                  // positive
                                                                                                                  // raw
                                                                                                                  // distance
                                                                                                                  // between
                                                                                                                  // the
                                                                                                                  // angles
        double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle

        if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
            signedDiff = ((2 * Math.PI) - modDiff); // full circle minus the angle
            if (desiredAngle > currentAngle)
                signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
        } else {
            signedDiff = modDiff;
            if (currentAngle > desiredAngle)
                signedDiff = signedDiff * -1;
        }
        return signedDiff;
    }

    public void stop() {
        m_driveMotor.set(0);
        //m_turningMotor.set(0);
    }
}
