package frc.robot.SubSystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    public final String name;

    public final Translation2d translation;
    public final Rotation2d angle_offset;

    private MotorController angle_motor;
    private MotorController drive_motor;
    private CANCoder angle_sensor;

    private PIDController angle_pid;
    private SimpleMotorFeedforward angle_ff;
    private Constraints angle_constraints;

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
     * @param angle_pid    Module angle PID object
     * @param angle_ff     Module angle FeedForward object
     * @param angle_prof   Module angle Trapezoidal profile object
     */
    public SwerveModule(String name, Translation2d translation, Rotation2d angle_offset, MotorController angle_motor,
            MotorController drive_motor, CANCoder angle_sensor, PIDController angle_pid,
            SimpleMotorFeedforward angle_ff, Constraints angle_constraints) {

        this.name = name;
        this.translation = translation;
        this.angle_offset = angle_offset;

        this.angle_motor = angle_motor;
        this.drive_motor = drive_motor;
        this.angle_sensor = angle_sensor;

        this.angle_pid = angle_pid;
        this.angle_ff = angle_ff;
        this.angle_constraints = angle_constraints;
    }

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

    /**
     * Module periodic Function
     */
    @Override
    public void periodic() {

        // Get Module Angle
        Rotation2d angle = get_angle();
        double angle_rate = angle_sensor.getVelocity();

        // Get Module Error
        Rotation2d error = get_error(angle);
        var error_abs = Math.abs(error.getDegrees());
        boolean speed_inv = false;

        Rotation2d error_pos = get_error(angle.plus(Rotation2d.fromDegrees(360)));
        Rotation2d error_pos_inv = get_error(angle.plus(Rotation2d.fromDegrees(180)));
        Rotation2d error_neg = get_error(angle.minus(Rotation2d.fromDegrees(360)));
        Rotation2d error_neg_inv = get_error(angle.minus(Rotation2d.fromDegrees(180)));

        if (error_abs > Math.abs(error_pos.getDegrees())) {
            error = error_pos;
            speed_inv = false;
        }

        if (error_abs > Math.abs(error_pos_inv.getDegrees())) {
            error = error_pos;
            speed_inv = true;
        }

        if (error_abs > Math.abs(error_neg.getDegrees())) {
            error = error_neg;
            speed_inv = false;
        }

        if (error_abs > Math.abs(error_neg_inv.getDegrees())) {
            error = error_neg_inv;
            speed_inv = true;
        }

        // Calculate target angle rate
        double target_angle_rate = 0;
        if (!at_angle(error)) {
            // Get Module Angle Target
            Rotation2d angle_target = angle.plus(error);

            // Calculate angle speed
            State target_state = new State(angle_target.getDegrees(), 0);
            State initial_state = new State(angle.getDegrees(), angle_rate);
            TrapezoidProfile profile = new TrapezoidProfile(angle_constraints, target_state, initial_state);

            State target_angle_state = profile.calculate(0);

            target_angle_rate = target_angle_state.velocity;
        }
        // TODO Update target angle to Driver Station
        // TODO Update target angle rate to Driver Station
        // TODO Update actual angle to Driver Station
        // TODO Update actual angle rate to Driver Station

        // Calculate
        var angle_pid_value = angle_pid.calculate(angle_rate, target_angle_rate);
        var angle_ff_value = angle_ff.calculate(target_angle_rate);

        angle_motor.set(angle_pid_value + angle_ff_value);

        // Set Module Speed
        // TODO Allow speed to be set in meters per second
        // TODO Update set speed on Drive Station
        // TODO Update Current speed on Driver Station

        if(speed_inv) {
            drive_motor.set(-this.speed_target);
        } else {
            drive_motor.set(this.speed_target);
        }

    }

}
