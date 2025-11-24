package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    private final LightsSubsystem lightsSubsystem;

    // Coral Subsystem Motors
//    private final TalonSRX        elevatorMotor             = new TalonSRX(CoralConstants.ELEVATOR_MOTOR_CAN_ID);
//    private final TalonSRX        armMotor                  = new TalonSRX(CoralConstants.ARM_MOTOR_CAN_ID);
//    private final TalonSRX        intakeMotor               = new TalonSRX(CoralConstants.INTAKE_MOTOR_CAN_ID);

    private double              elevatorSpeed                       = 0;
    private double              armSpeed                            = 0;
    private double              intakeSpeed                         = 0;

    private double              elevatorEncoderOffset               = 0;
    private double              elevatorEncoder                     = 0;

    // Simulation constants
    // Full speed up: the elevator will raise 60 inches in 2 seconds with a loop time of 20ms.
    private static final double ELEVATOR_MAX_UP_DISTANCE_PER_LOOP   = 60 * .02 / 2;
    // Full speed down: the elevator will lower in 1.5 seconds.
    private static final double ELEVATOR_MAX_DOWN_DISTANCE_PER_LOOP = 60 * .02 / 1.5;

    public CoralSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;
    }

    /*
     * Elevator Routines
     */

    public void setElevatorSpeed(double speed) {

        this.elevatorSpeed = speed;

        checkSafety();

        // elevatorMotor.set(ControlMode.PercentOutput, elevatorSpeed);
    }

    public boolean isElevatorAtLowerLimit() {

        // This method should check a sensor.
        if (getElevatorEncoder() <= 0) {
            return true;
        }

        return false;
    }

    public boolean isElevatorAtUpperLimit() {

        // This method should check a sensor.
        if (getElevatorEncoder() >= 60) {
            return true;
        }

        return false;
    }

    public double getElevatorEncoder() {
        // This method should read an encoder
        return elevatorEncoder + elevatorEncoderOffset;
    }

    public void resetElevatorEncoder() {
        setElevatorEncoder(0);
    }

    public void setElevatorEncoder(double encoderValue) {

        elevatorEncoderOffset = 0;
        elevatorEncoderOffset = -getElevatorEncoder() + encoderValue;
    }

    /*
     * Arm Routines
     * FIXME: Make arm routines similar to the elevator routines
     */
    public void setArmSpeed() {

    }

    /*
     * Intake Routines
     * FIXME: Make intake routines similar to the elevator routines
     * NOTE: the intake will not have encoders or limit switches.
     */



    /*
     * Periodic routines
     */
    @Override
    public void periodic() {

        // FIXME: replace the simulation when the robot is ready.
        simulate();

        checkSafety();

        // FIXME: Add a call to the lights subsystem to show the current speed or height
        // lightsSubsystem.setCoral..();

        SmartDashboard.putNumber("Coral Elevator Motor", elevatorSpeed);
        SmartDashboard.putNumber("Coral Arm Motor", armSpeed);
        SmartDashboard.putNumber("Coral Intake Motor", intakeSpeed);

        // FIXME: what else should we put on the SmartDashboard
    }

    private void simulate() {

        // This loop will be called every 20 ms, 50 times per second

        // Move the elevator up or down depending on the direction of the motor speed
        // The elevator will fall faster than it will lift.
        if (elevatorSpeed > 0) {
            elevatorEncoder += ELEVATOR_MAX_UP_DISTANCE_PER_LOOP * elevatorSpeed;
        }
        if (elevatorSpeed < 0) {
            elevatorEncoder += ELEVATOR_MAX_DOWN_DISTANCE_PER_LOOP * elevatorSpeed;
        }
    }

    private void checkSafety() {

        if (isElevatorAtLowerLimit()) {

            if (elevatorSpeed < 0) {
                elevatorSpeed = 0;
                // Directly set the motor speed, do not call the setter method (recursive loop)
                // elevatorMotor.set(ControlMode.PercentOutput, 0);
                resetElevatorEncoder();
            }
        }

        // FIXME: add an upper limit check
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Elevator: speed ").append(elevatorSpeed)
            .append(" height ").append(getElevatorEncoder()).append("in");

        return sb.toString();
    }
}
