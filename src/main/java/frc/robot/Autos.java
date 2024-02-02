package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Autos {

    //FIXME: just rewrite all of this terribleness
    private final DriveSubsystem m_Drivetrain;
    private final Feeder m_Feeder;
    private final Intake m_Intake;
    private final Pneumatics m_Pneumatics;
    private final Shooter m_Shooter;
    private SendableChooser<Command> autoChooser;
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drive, Feeder feed, Intake intake, Pneumatics pneumatic, Shooter shooter){
        m_Drivetrain = drive;
        m_Feeder = feed;
        m_Intake = intake;
        m_Pneumatics = pneumatic;
        m_Shooter = shooter;
        autoChooser = new SendableChooser<Command>();
    
        // autoChooser = AutoBuilder.buildAutoChooser(); 
        

        autoChooser.addOption("Drivetrain Characterization", new FeedForwardCharacterization(m_Drivetrain, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
            m_Drivetrain::runCharacterizationVolts, m_Drivetrain::getCharacterizationVelocity));
        autoChooser.addOption("Shooter Characterization", new FeedForwardCharacterization(m_Shooter, true, new FeedForwardCharacterizationData("Shooter"),
                m_Shooter::runCharacterizationVolts , m_Shooter::getCharacterizationVelocity));
    
        autoChooser.addOption("Basic Test Auto", AutoBuilder.buildAuto("Basic Test Auto"));
        autoChooser.addOption("4Ring", AutoBuilder.buildAuto("4Ring"));

        SmartDashboard.putData("Autos", autoChooser);
        //autoTab.add(autoChooser); sure
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
