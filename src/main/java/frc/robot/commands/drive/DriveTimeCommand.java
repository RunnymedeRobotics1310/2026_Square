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

        // "this" keyword is just for referencing the variable that is declared above
        this.driveSubsystem = driveSubsystem; // gives us access to the drive subsystem
        this.duration = duration; // how long the command will run for

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() { // runs when the command starts, kinda like the constructor
        logCommandStart();
        timer.restart();
    }

    @Override
    public void execute() { // runs evey 20 milliseconds
        // the subsystem takes care of the motor inversions and driving the followers
        driveSubsystem.setMotorSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() { // FIXME: when should the command stop?
        // hint: use timer.get() (you will need to read the docs to know what it gives you)
        return false; // once this returns true, then the command stops
    }

    @Override
    public void end(boolean interrupted) { // FIXME: what should happen when the command stops?

    }

}