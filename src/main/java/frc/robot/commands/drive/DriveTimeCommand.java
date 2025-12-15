package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeCommand extends LoggingCommand {

    private final DriveSubsystem driveSubsystem;
    private final Timer timer = new Timer();
    private final double duration;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     * @param duration Time to drive for
     */
    public DriveTimeCommand(DriveSubsystem driveSubsystem, double duration) {

        this.driveSubsystem = driveSubsystem;
        this.duration = duration;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();

        timer.restart();
    }

    @Override
    public void execute() {

        driveSubsystem.setMotorSpeeds(0.2, 0.2);
    }

    @Override
    public boolean isFinished() {

        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {

        driveSubsystem.setMotorSpeeds(0.0, 0.0);
    }

}