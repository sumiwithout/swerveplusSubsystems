package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;
public class FeedLauncher extends CommandBase{
    private Timer m_timer;
    IntakeSubsystem m_intake;
    LauncherSubsystem m_launcher;
    // help
    public FeedLauncher(IntakeSubsystem intake, LauncherSubsystem _launcher){
        m_intake = intake;
        m_launcher = _launcher;
        addRequirements(m_intake, m_launcher);
    }
    @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            m_intake.setPower(1);
            m_launcher.runLauncher();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            m_intake.setPower(0);
          }
}
