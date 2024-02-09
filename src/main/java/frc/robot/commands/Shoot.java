package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
public class Shoot extends Command {
private final Shooter m_ShooterSubSystem;
    public Shoot(Shooter shooterSub){
    m_ShooterSubSystem = shooterSub;
    addRequirements(m_ShooterSubSystem);
    }

    /* WHAT WE NEED FOR SHOOTER
     * SHOOT WHEN LIMELIGHT POSE == IDEAL POSE *
     * or, what we could do is:
     * IF INFRONT OF IDEAL POSE
     *  LOWER SHOT PERCENT POWER. 
     * 
     * 
     * 
     * 
     * 
    */
}
