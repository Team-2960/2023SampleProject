package frc.robot.SubSystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PosController;
import frc.robot.util.RateController;

public class SwerveModule extends SubsystemBase {
    public class AngleTarget {
        public Rotation2d target;
        public boolean inv_drive;

        public AngleTarget(Rotation2d target, boolean inv_drive) {
            this.target = target;
            this.inv_drive = inv_drive;
        }
    }

    public final String name;

    public final Translation2d translation;
    public final Rotation2d angle_offset;

    private MotorController angle_motor;
    private MotorController drive_motor;
    private CANCoder angle_sensor;

    private RateController drive_rate_ctrl;
    private PosController angle_pos_ctrl;

    private Rotation2d angle_target = Rotation2d.fromDegrees(0);
    private double speed_target = 0;

    private Rotation2d angle_tol = Rotation2d.fromDegrees(1);

    /**
     * Constructor
     * 
     * @param name         Name of the module
     * @param translation  Module position relative to robot origin
     * @param angle_offset Module angle sensor zero offset
     * @param angle_motor  Module angle motor
     * @param drive_motor  Module drive motor
     * @param angle_sensor Module angle sensor. Should be configured to have a range
     *                     of [0,360) degrees with the positive direction
     *                     counter-clockwise
     * @param drive_ff     Module drive FeedForwrd parameters
     * @param angle_pid    Module angle PID parameters
     * @param angle_ff     Module angle FeedForward parameters
     * @param angle_prof   Module angle Trapezoidal profile object
     */
    public SwerveModule(String name, Translation2d translation, Rotation2d angle_offset, MotorController angle_motor,
            MotorController drive_motor, CANCoder angle_sensor, RateController.FFParam drive_ff,
            RateController.PIDParam angle_pid, RateController.FFParam angle_ff, PosController.PosParam angle_pos_param) {

        this.name = name;
        this.translation = translation;
        this.angle_offset = angle_offset;

        this.angle_motor = angle_motor;
        this.drive_motor = drive_motor;
        this.angle_sensor = angle_sensor;

        this.drive_rate_ctrl = new RateController(drive_ff);
        this.angle_pos_ctrl = new PosController(new RateController(angle_pid, angle_ff), angle_pos_param);
    }

    /**
     * Sets the target state of the swerve module
     * 
     * @param state target state of the swerve module
     */
    public void set_target(SwerveModuleState state) {
        set_target(state.angle, state.speedMetersPerSecond);
    }

    /**
     * Sets module target angle and speed
     * 
     * @param angle_target Module target angle
     * @param speed_target Module target speed
     */
    public void set_target(Rotation2d angle_target, double speed_target) {
        set_angle(angle_target);
        set_speed(speed_target);
    }

    /**
     * Sets module target angle. Angle will be modified to be in the range [0,360)
     * degrees.
     * 
     * @param target
     */
    public void set_angle(Rotation2d target) {
        // Ensure target is less than 360 degrees
        while (target.getDegrees() > 360)
            target = target.minus(Rotation2d.fromDegrees(360));

        while (target.getDegrees() <= 0)
            target = target.plus(Rotation2d.fromDegrees(0));

        this.angle_target = target;
    }

    /**
     * Sets module target speed
     * 
     * @param speed_target
     */
    public void set_speed(double target) {
        this.speed_target = target;
    }

    /**
     * Sets the angle tolerance for the module
     * 
     * @param tol module angle tolerance
     */
    public void set_angle_tol(Rotation2d tol) {
        this.angle_tol = tol;
    }

    /**
     * Gets the current module angle
     * 
     * @return current module angle
     */
    public Rotation2d get_angle() {
        return Rotation2d.fromDegrees(angle_sensor.getAbsolutePosition());
    }

    /**
     * Gets the current module angle error
     * 
     * @param angle current module angle
     * @return current module angle error
     */
    public Rotation2d get_error(Rotation2d angle) {
        return angle_target.minus(angle);
    }

    /**
     * Checks if the module angle is within tolerance of the target angle
     * 
     * @return true if the module angl eis within tolerance of the target angle
     */
    public boolean at_angle() {
        return at_angle(get_angle());
    }

    /**
     * Checks if the module angle is within tolerance of the target angle
     * 
     * @param angle Current module angle
     * @return true if the module angl eis within tolerance of the target angle
     */
    public boolean at_angle(Rotation2d angle) {
        return Math.abs(get_error(angle).getDegrees()) < angle_tol.getDegrees();
    }

    public static AngleTarget min_angle_target(Rotation2d current, Rotation2d target) {
        Rotation2d error = target.minus(current);
        var error_abs = Math.abs(error.getDegrees());
        boolean inv_drive = false;

        Rotation2d error_pos = get_error(angle.plus(Rotation2d.fromDegrees(360)));
        Rotation2d error_pos_inv = get_error(angle.plus(Rotation2d.fromDegrees(180)));
        Rotation2d error_neg = get_error(angle.minus(Rotation2d.fromDegrees(360)));
        Rotation2d error_neg_inv = get_error(angle.minus(Rotation2d.fromDegrees(180)));

        if (error_abs > Math.abs(error_pos.getDegrees())) {
            error = error_pos;
            inv_drive = false;
        }

        if (error_abs > Math.abs(error_pos_inv.getDegrees())) {
            error = error_pos;
            inv_drive = true;
        }

        if (error_abs > Math.abs(error_neg.getDegrees())) {
            error = error_neg;
            inv_drive = false;
        }

        if (error_abs > Math.abs(error_neg_inv.getDegrees())) {
            error = error_neg_inv;
            inv_drive = true;
        }

        return new AngleTarget(current.plus(mod_error.error), inv_drive);
    }

    /**
     * Module periodic Function
     */
    @Override
    public void periodic() {

        // Get current Module state
        Rotation2d angle = get_angle();
        double angle_rate = ;

        // Calculate optimial target angle
        AngleTarget angle_target = min_angle_target(angle, this.angle_target);

        // Set Angle motor power
        angle_volt = angle_pos_ctrl.calculate(
            new ControlState(angle.getRadians(), angle_sensor.getVelocity()),   // Current Angle State
            new ControlState(angle_target.target.getRadians(), 0)               // Target Angle State
        );

        angle_motor.setVoltage(angle_volt);
        

        // set motor speed
        drive_volt = drive_rate_ctrl.calculate(
            new ControlState(drive_motor.getVelocity()),
            new ControlState((angle_target.inv_drive ? 1 : -1) * this.speed_target);
        )

        drive_motor.setVoltage(drive_volt)

        // TODO Update Drivers station outputs

    }

}
