package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.GameController;

/**
 * The DriverController exposes all driver functions
 *
 * <p>Extend SubsystemBase in order to have a built in periodic call to support SmartDashboard
 * updates
 */
public class OperatorInput extends SubsystemBase {

  private final GameController driverController;

  /**
   * Construct an OperatorInput class that is fed by a DriverController and optionally an
   * OperatorController.
   */
  public OperatorInput() {

    driverController =
        new GameController(
            OperatorInputConstants.DRIVER_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);
  }

  /*
   * Cancel Command support
   * Do not end the command while the button is pressed
   */
  public boolean isCancel() {
    return driverController.getStartButton();
  }
}
