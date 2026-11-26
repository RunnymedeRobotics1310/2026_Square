package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.Constants.BallConstants.*;

public class BallSubsystem {

  private final DoubleSolenoid intakeSolenoid;
  private final DoubleSolenoid winchSolenoid;

  private final TalonSRX intakeMotor;

  private final DigitalInput catapultSensor;

  public BallSubsystem(){
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE_OUT_CHANNEL, INTAKE_IN_CHANNEL);
    winchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_ENGAGE_CHANNEL, WINCH_DISENGAGE_CHANNEL);

    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    winchSolenoid.set(DoubleSolenoid.Value.kReverse);

    intakeMotor = new TalonSRX(INTAKE_MOTOR_CANID);

    catapultSensor = new DigitalInput(0);
  }

  /**
   * Intake methods
   */
  public void intake(){
    /*
    check if launching/not reloaded
    extend, run motors
     */

    if(isCatapultExtended()) return;

    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopIntake(){
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Launch methods
   */
  public void launch(){
      /*
      move intake out of way. Priority over intake
      release winch
      move intake back
       */

  }

  /**
   * Reload methods
   */
  public void reload(){
      /*
      winch down
       */
  }

/**
 * Other methods
 */
public void stopAll(){
    stopIntake();
}


  /**
   * Getters/setters
   */

  public boolean isCatapultExtended(){
    return catapultSensor.get();
  }

}
