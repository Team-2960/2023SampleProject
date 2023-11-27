package frc.robot.SubSystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain inst;
    /** < Statically initialized instance of the Drivetrain class */

    private SwerveModule lf_module;
    /** < Left Front Swerve Module */
    private SwerveModule rf_module;
    /** < Right Front Swerve Module */
    private SwerveModule lr_module;
    /** < Left Rear Swerve Module */
    private SwerveModule rr_module;
    /** < Right Rear Swerve Module */

    private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(4);
    /** < List of Swerve Modules */

    private SwerveDriveKinematics kinematics;
    /** < Swerve Module Kinematics Module */

    private AHRS imu;
    /** < IMU */

    ChassisSpeeds speeds = new ChassisSpeeds();
    /** < Chassis Speeds object */

    Constants constants = Constants.get_instance();

    /** < Instance of constants object */

    /**
     * Constructor
     */
    private Drivetrain() {

        // Initialize Swerve Modules
        lf_module = new SwerveModule("Left Front Swerve",
                new Translation2d(-constants.module_offset, constants.module_offset), new Rotation2d(0),
                new WPI_TalonFX(1), new WPI_TalonFX(2), new CANCoder(3),
                constants.drive_ff, constants.drive_angle_pid,
                constants.drive_angle_ff, constants.angle_constraints);

        rf_module = new SwerveModule("Right Front Swerve",
                new Translation2d(constants.module_offset, constants.module_offset), new Rotation2d(0),
                new WPI_TalonFX(1), new WPI_TalonFX(2), new CANCoder(3),
                constants.drive_ff, constants.drive_angle_pid,
                constants.drive_angle_ff, constants.angle_constraints);

        lr_module = new SwerveModule("Left Rear Swerve",
                new Translation2d(-constants.module_offset, -constants.module_offset), new Rotation2d(0),
                new WPI_TalonFX(1), new WPI_TalonFX(2), new CANCoder(3),
                constants.drive_ff, constants.drive_angle_pid,
                constants.drive_angle_ff, constants.angle_constraints);

        rr_module = new SwerveModule("Right Rear Swerve",
                new Translation2d(constants.module_offset, -constants.module_offset), new Rotation2d(0),
                new WPI_TalonFX(1), new WPI_TalonFX(2), new CANCoder(3),
                constants.drive_ff, constants.drive_angle_pid,
                constants.drive_angle_ff, constants.angle_constraints);

        modules.add(lf_module);
        modules.add(rf_module);
        modules.add(lr_module);
        modules.add(rr_module);

        // Initialize IMU
        imu = new AHRS(SPI.Port.kMXP);

        // Initialize Kinematics
        Translation2d[] translations = new Translation2d[modules.size()];

        for (int i = 0; i < modules.size(); i++)
            translations[i] = modules.get(i).translation;

        kinematics = new SwerveDriveKinematics(translations);
    }

    /**
     * Retrieve the current angle of the robot from the IMU
     * 
     * @return Current angle of the robot
     */
    public Rotation2d get_angle() {
        // TODO Allow robot angle to be reset
        return Rotation2d.fromDegrees(imu.getAngle());
    }

    /**
     * Set the robots X and Y velocity.
     * 
     * @param x_vel X velocity in meters per second
     * @param y_vel Y velocity in meters per second
     */
    public void set(double x_vel, double y_vel) {
        speeds.vxMetersPerSecond = x_vel;
        speeds.vyMetersPerSecond = x_vel;
    }

    /**
     * Set the robot's X, Y, and angular velocity.
     * 
     * @param x_vel X velocity in meters per second
     * @param y_vel Y velocity in meters per second
     * @param a_vel angular velocity in degrees per second
     */
    public void set(double x_vel, double y_vel, double a_vel) {
        set(x_vel, y_vel);
        set_angle_rate(a_vel);
    }

    /**
     * Set the robots velocity vector
     * 
     * @param vel     Velocity in meters per second
     * @param heading Heading angle relative to the field
     */
    public void set(double vel, Rotation2d heading) {
        set(vel * heading.getCos(), vel * heading.getSin());
    }

    /**
     * Set the robots velocity vector
     * 
     * @param vel     Velocity in meters per second
     * @param heading Heading angle relative to the field
     * @param a_vel   Angular Rate of the robot
     */
    public void set(double vel, Rotation2d heading, double a_vel) {
        set(vel * heading.getCos(), vel * heading.getSin(), a_vel);
    }

    /**
     * Set the robot's angular rate
     * 
     * @param a_vel Angular Rate of the robot
     */
    public void set_angle_rate(double a_vel) {
        speeds.omegaRadiansPerSecond = a_vel * Math.PI / 180;
    }

    /**
     * Subsystem Periodic function
     */
    @Override
    public void periodic() {
        // Set Module Speeds
        var speeds_fr = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, get_angle());
        var states = kinematics.toSwerveModuleStates(speeds_fr);

        // TODO Offset angle rate to compensate for error from target angle rate

        for (int i = 0; i < Math.min(modules.size(), states.length); i++) {
            modules.get(i).set_target(states[i]);
        }
    }

    /**
     * Static initializer
     * 
     * @return instance of the drivetrain object
     */
    public static Drivetrain get_instance() {
        if (inst == null) {
            inst = new Drivetrain();
        }

        return inst;
    }

}
