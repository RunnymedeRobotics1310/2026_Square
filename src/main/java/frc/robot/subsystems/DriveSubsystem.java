package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.studica.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


    // The motors on the left side of the drive.
    private final TalonSRX leftPrimaryMotor = new TalonSRX(DriveConstants.LEFT_MOTOR_CAN_ID);
    private final TalonSRX leftFollowerMotor = new TalonSRX(DriveConstants.LEFT_MOTOR_CAN_ID+1);

    // The motors on the right side of the drive.
    private final TalonSRX rightPrimaryMotor = new TalonSRX(DriveConstants.RIGHT_MOTOR_CAN_ID);
    private final TalonSRX rightFollowerMotor = new TalonSRX(DriveConstants.RIGHT_MOTOR_CAN_ID + 1);

    private final DigitalInput targetSensor = new DigitalInput(0);

    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput ultrasonicDistanceSensor = new AnalogInput(0);

    private final double ULTRASONIC_M = (609.6 - 30.5) / (2.245 - .12);
    private final double ULTRASONIC_B = 609.6 - ULTRASONIC_M * 2.245;


    private double leftSpeed = 0;
    private double rightSpeed = 0;

    private double leftEncoderOffset = 0;
    private double rightEncoderOffset = 0;


    /*
     * Simulation fields
     */
    private Field2d field = null;
    private DifferentialDrivetrainSim drivetrainSim = null;
    private double simAngle = 0;
    private double simLeftEncoder = 0;
    private double simRightEncoder = 0;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {


        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_INVERTED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_INVERTED);

        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_INVERTED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_INVERTED);


        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        leftFollowerMotor.follow(leftPrimaryMotor);

// todo: fixme: set inversion for right motors

        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightPrimaryMotor);

        // Add the field elements for robot simulations
        if (RobotBase.isSimulation()) {

            field = new Field2d();
            SmartDashboard.putData("Field", field);

            drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                    KitbotMotor.kDoubleNEOPerSide, // Double NEO per side
                    KitbotGearing.k10p71, // 10.71:1
                    KitbotWheelSize.kSixInch, // 6" diameter wheels.
                    null // No measurement noise.
            );
        }
    }

    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();

        double distanceCm = ULTRASONIC_M * ultrasonicVoltage + ULTRASONIC_B;

        return Math.round(distanceCm);
    }


    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderValue() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceCm() {

        return getAverageEncoderValue() * DriveConstants.CM_PER_ENCODER_COUNT;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return simLeftEncoder + leftEncoderOffset;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return simRightEncoder + rightEncoderOffset;
    }

    /**
     * Resets the drive encoders to zero.
     */
    public void resetEncoders() {

        // Reset the offsets so that the encoders are zeroed.
        leftEncoderOffset = 0;
        leftEncoderOffset = -getLeftEncoder();

        rightEncoderOffset = 0;
        rightEncoderOffset = -getRightEncoder();
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);

        // NOTE: The follower motors are set to follow the primary
        // motors
    }

    /**
     * Safely stop the subsystem from moving
     */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    public boolean isTargetDetected() {
        return !targetSensor.get();
    }

    @Override
    public void periodic() {


        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Right Encoder", Math.round(getRightEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Left Encoder", Math.round(getLeftEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Avg Encoder", Math.round(getAverageEncoderValue() * 100) / 100d);
        SmartDashboard.putNumber("Distance (cm)", Math.round(getEncoderDistanceCm() * 10) / 10d);

        SmartDashboard.putNumber("Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
        SmartDashboard.putNumber("Ultrasonic Distance (cm)", getUltrasonicDistanceCm());

    }

    @Override
    public void simulationPeriodic() {

        if (RobotController.isSysActive()) {

            // When the robot is enabled, calculate the position
            // Set the inputs to the system.
            drivetrainSim.setInputs(
                    leftSpeed * RobotController.getInputVoltage(),
                    rightSpeed * RobotController.getInputVoltage());

            // Advance the model by 20 ms. Note that if you are running this
            // subsystem in a separate thread or have changed the nominal timestep
            // of TimedRobot, this value needs to match it.
            drivetrainSim.update(0.02);

            // Move the robot on the simulated field
            field.setRobotPose(drivetrainSim.getPose());
        } else {
            // When the robot is disabled, allow the user to move
            // the robot on the simulation field.
            drivetrainSim.setPose(field.getRobotPose());
        }

        // Update the gyro simulation offset
        // NOTE: the pose has the opposite rotational direction from the system
        // pose degrees are counter-clockwise positive. weird.
        simAngle = -drivetrainSim.getPose().getRotation().getDegrees();

        // Update the encoders with the simulation offsets.
        simLeftEncoder = drivetrainSim.getLeftPositionMeters() * 100 / DriveConstants.CM_PER_ENCODER_COUNT;
        simRightEncoder = drivetrainSim.getRightPositionMeters() * 100 / DriveConstants.CM_PER_ENCODER_COUNT;
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
                .append(", Drive dist ").append(Math.round(getEncoderDistanceCm() * 10) / 10d).append("cm");

        return sb.toString();
    }
}
