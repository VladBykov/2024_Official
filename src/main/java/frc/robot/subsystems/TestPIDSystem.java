package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class TestPIDSystem extends SubsystemBase {
    /**
     * This a test class to test PID controllers
     * 
     *      drive system - testing for standard PID controll
     * 
     */
    public PIDController         m_driveController;
    public ProfiledPIDController m_turnController;

    private double      m_driveCurrent  = 0.0;
    private double      m_driveDesired  = m_driveCurrent;
    
    private Rotation2d  m_turnThetaCurrent = new Rotation2d(0);
    private Rotation2d  m_turnThetaDesired = m_turnThetaCurrent;

    // PID values
    public double m_kP = 0.5;
    public double m_kI = 0.0;
    public double m_kD = 0.0;

    protected static final double s_maxTurnVel  = 30.0; // degrees / sec
    protected static final double s_maxTurnAcc  = 10.0; // degrees / sec^2

    public TestPIDSystem()
    {
        // create the PID controllers
        m_driveController = new PIDController(
            m_kP, m_kI, m_kD
        );
        m_turnController  = new ProfiledPIDController(
            m_kP, m_kI, m_kD, 
            new TrapezoidProfile.Constraints(
                s_maxTurnVel / TimedRobot.kDefaultPeriod, 
                s_maxTurnAcc / TimedRobot.kDefaultPeriod
            ) // need to calculate out the periodic constraint that we are using in Robot.java's TimedRobot class
        );
        
        // Need to reset the controllers upon initializing them
        m_driveController.reset();
        m_turnController.reset(m_turnThetaCurrent.getDegrees());

        this.updateCurrentDashboard();
        this.updateDesiredDashboard();
    }

    public SwerveModuleState getCurrentState()
    {
        return new SwerveModuleState(
            m_driveCurrent,
            m_turnThetaCurrent
        );
    }
    public SwerveModuleState getDesiredState()
    {
        return new SwerveModuleState(
            m_driveDesired,
            m_turnThetaDesired
        );
    }

    public void setCurrentState(SwerveModuleState state)
    {
        m_driveCurrent     = state.speedMetersPerSecond;
        m_turnThetaCurrent = state.angle;
    }

    public void setDesiredState(SwerveModuleState state)
    {
        m_driveDesired     = state.speedMetersPerSecond;
        m_turnThetaDesired = state.angle;
    }

    /**
     * When you are performing PID control, the PIDController.calculate 
     * fuction is used to determine how much we need to update the current
     * state. 
     * Therefore, if we want to change the current velocity, we would need 
     * to update the current velocity by a changed value.
     * 
     * This is NOT the same as what is used based on SparkMaxPIDController
     * 
     * @param stateUpdate - the update that is calculated from PID
     */
    public void updateState(SwerveModuleState stateUpdate)
    {
        m_driveCurrent     += stateUpdate.speedMetersPerSecond;
        m_turnThetaCurrent  = Rotation2d.fromRadians(
            m_turnThetaCurrent.getRadians() + stateUpdate.angle.getRadians()
        );
    }

    @Override
    public void periodic()
    {
        // Update current desired from SmartDashboard
        double driveDesired = SmartDashboard.getNumber("Test PID/Drive Desired (mps)", m_driveDesired);
        double turnDesired  = SmartDashboard.getNumber("Test PID/Turn Desired (degs)", m_turnThetaDesired.getDegrees());

        if (driveDesired != m_driveDesired) { 
            m_driveDesired = driveDesired; 
            System.out.println("Update desired drive to:" + m_driveDesired);
        }
        if (turnDesired  != m_turnThetaCurrent.getDegrees()) {
            m_turnThetaDesired = Rotation2d.fromDegrees(turnDesired); 
            System.out.println("Update desired turn to:" + m_turnThetaDesired.getDegrees());
        }

        // Calculate the update to the current state
        SwerveModuleState currentState = getCurrentState();
        SwerveModuleState desiredState = getDesiredState();

        double driveInput     = m_driveController.calculate(
            currentState.speedMetersPerSecond, 
            desiredState.speedMetersPerSecond
        );
        Rotation2d turnInput  = Rotation2d.fromDegrees(
            m_turnController.calculate(
                currentState.angle.getDegrees(), 
                desiredState.angle.getDegrees()
            )
        );

        if (Math.abs(driveInput) < 1e-6)
            driveInput = 0;

        // update dashboard
        this.updateCurrentDashboard();
        SmartDashboard.putNumber("Test PID/Drive Input (mps)", driveInput);
        SmartDashboard.putNumber("Test PID/Turn Input (degs)", turnInput.getDegrees());

        this.updateState(new SwerveModuleState(driveInput, turnInput));

        super.periodic();

    }

    public void updateCurrentDashboard()
    {
        SwerveModuleState currentState = getCurrentState();
        SmartDashboard.putNumber("Test PID/Drive Current (mps)", currentState.speedMetersPerSecond);
        SmartDashboard.putNumber("Test PID/Turn Current (degs)", currentState.angle.getDegrees());


        SmartDashboard.putNumberArray(
            "PID constants k(P, I, D)", 
            new Double[] {m_kP, m_kI, m_kD}
        );
    }

    public void updateDesiredDashboard()
    {
        SwerveModuleState desiredState = getDesiredState();
        SmartDashboard.putNumber("Test PID/Drive Desired (mps)", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Test PID/Turn Desired (degs)", desiredState.angle.getDegrees());

    }
}
