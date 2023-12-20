package frc.robot.SubSystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Sensors.PosSensor_CTRETalonFX;
import frc.robot.util.Sensors.PosSensor_RevSparkMax;

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
        // LF Swerve Module
        Translation2d lf_pos = new Translation2d(-constants.module_offset, constants.module_offset);
        WPI_TalonFX lf_drive_motor = new WPI_TalonFX(1);
        CANSparkMax lf_angle_motor = new CANSparkMax(2, MotorType.kBrushless);
        PosSensor_CTRETalonFX lf_drive_sensor = new PosSensor_CTRETalonFX(lf_drive_motor);
        PosSensor_RevSparkMax lf_angle_sensor = new PosSensor_RevSparkMax(
                lf_angle_motor.getAlternateEncoder(2048),
                lf_angle_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                0.0);

        lf_module = new SwerveModule("Left Front Swerve", lf_pos, lf_drive_motor, lf_angle_motor, lf_drive_sensor,
                lf_angle_sensor, constants.drive_ff, constants.drive_angle_pid, constants.drive_angle_ff,
                constants.angle_pos_param);

        // RF Swerve Module
        Translation2d rf_pos = new Translation2d(-constants.module_offset, constants.module_offset);
        WPI_TalonFX rf_drive_motor = new WPI_TalonFX(1);
        CANSparkMax rf_angle_motor = new CANSparkMax(2, MotorType.kBrushless);
        PosSensor_CTRETalonFX rf_drive_sensor = new PosSensor_CTRETalonFX(rf_drive_motor);
        PosSensor_RevSparkMax rf_angle_sensor = new PosSensor_RevSparkMax(
                rf_angle_motor.getAlternateEncoder(2048),
                rf_angle_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                0.0);

        rf_module = new SwerveModule("Right Front Swerve", rf_pos, rf_drive_motor, rf_angle_motor, rf_drive_sensor,
                rf_angle_sensor, constants.drive_ff, constants.drive_angle_pid, constants.drive_angle_ff,
                constants.angle_pos_param);

        // LR Swerve Module
        Translation2d lr_pos = new Translation2d(-constants.module_offset, constants.module_offset);
        WPI_TalonFX lr_drive_motor = new WPI_TalonFX(1);
        CANSparkMax lr_angle_motor = new CANSparkMax(2, MotorType.kBrushless);
        PosSensor_CTRETalonFX lr_drive_sensor = new PosSensor_CTRETalonFX(lr_drive_motor);
        PosSensor_RevSparkMax lr_angle_sensor = new PosSensor_RevSparkMax(
                lr_angle_motor.getAlternateEncoder(2048),
                lr_angle_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                0.0);

        lr_module = new SwerveModule("Left Rear Swerve", lr_pos, lr_drive_motor, lr_angle_motor, lr_drive_sensor,
                lr_angle_sensor, constants.drive_ff, constants.drive_angle_pid, constants.drive_angle_ff,
                constants.angle_pos_param);

        // RR Swerve Module
        Translation2d rr_pos = new Translation2d(-constants.module_offset, constants.module_offset);
        WPI_TalonFX rr_drive_motor = new WPI_TalonFX(1);
        CANSparkMax rr_angle_motor = new CANSparkMax(2, MotorType.kBrushless);
        PosSensor_CTRETalonFX rr_drive_sensor = new PosSensor_CTRETalonFX(rr_drive_motor);
        PosSensor_RevSparkMax rr_angle_sensor = new PosSensor_RevSparkMax(
                rr_angle_motor.getAlternateEncoder(2048),
                rr_angle_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                0.0);

        rr_module = new SwerveModule("Right Rear Swerve", rr_pos, rr_drive_motor, rr_angle_motor, rr_drive_sensor,
                rr_angle_sensor, constants.drive_ff, constants.drive_angle_pid, constants.drive_angle_ff,
                constants.angle_pos_param);

        // Initialize Swerve Module Array
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
