package frc.robot.SubSystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OI extends SubsystemBase {
    private static OI inst;

    private XboxController driver_ctrl = new XboxController(0);
    private Drivetrain drivetrain = Drivetrain.get_instance();
    private Constants constants = Constants.get_instance();

    /**
     * Constructor
     */
    private OI() {

    }

    /**
     * Subsystem periodic function
     */
    public void periodic() {
        // Set speed
        double x_vel = driver_ctrl.getRawAxis(1) * constants.max_drive_speed;
        double y_vel = driver_ctrl.getRawAxis(0) * constants.max_drive_speed;
        double a_vel = driver_ctrl.getRawAxis(3) * constants.max_angle_rate;

        drivetrain.set(x_vel, y_vel, a_vel);
    }

    /**
     * Static Initializer
     * 
     * @return Instance of OI
     */
    public static OI get_instance() {
        if (inst == null) {
            inst = new OI();
        }

        return inst;
    }
}
