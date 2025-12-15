package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends LoggingCommand {

    private final DriveSubsystem driveSubsystem;
    private final OperatorInput  operatorInput;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param operatorInput which contains the drive mode selector.
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.operatorInput  = operatorInput;
        this.driveSubsystem = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        // Get the selected drive mode
        DriveMode driveMode          = operatorInput.getSelectedDriveMode();

        // Calculate the drive scaling factor based on the boost mode and the slow mode.
        double    driveScalingFactor = DriveConstants.DRIVE_SCALING_NORMAL;

        if (operatorInput.isBoost()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_BOOST;
        }
        if (operatorInput.isSlowDown()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_SLOW;
        }

        // If this is a tank drive robot, then the left and right speeds are set from the
        // joystick values.
        if (driveMode == DriveMode.TANK) {

            double leftSpeed  = operatorInput.getLeftSpeed();
            double rightSpeed = operatorInput.getRightSpeed();

            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

        }
        else {

            // 1. Deadband
            double speed = MathUtil.applyDeadband(operatorInput.getSpeed(), 0.1);
            double turn  = MathUtil.applyDeadband(operatorInput.getTurn(), 0.1);

            // 2. Square Inputs (Keep the sign!)
            speed = Math.copySign(speed * speed, speed);
            turn = Math.copySign(turn * turn, turn);

            // 3. Calculate Mix
            double left = speed + turn;
            double right = speed - turn;

            // 4. Clamp (Required for manual math)
            left = MathUtil.clamp(left, -1.0, 1.0);
            right = MathUtil.clamp(right, -1.0, 1.0);

            // 5. Output
            driveSubsystem.setMotorSpeeds(left, right);
        }

    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }

}