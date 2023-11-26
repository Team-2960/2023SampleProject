package frc.robot.SubSystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain inst;
    /** < Statically initialized instance of the Drivetrain class */

    private SwerveModule lf_module;
    private SwerveModule rf_module;
    private SwerveModule lr_module;
    private SwerveModule rr_module;

    private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(4);

    private SwerveDriveKinematics kinematics;

    private AHRS imu;

    ChassisSpeeds speeds = new ChassisSpeeds();

    public Drivetrain() {
        // TODO Initialize Swerve modules

        modules.add(lf_module);
        modules.add(rf_module);
        modules.add(lr_module);
        modules.add(rr_module);

        imu = new AHRS(SPI.Port.kMXP);

        // Initialize Kinematics
        Translation2d[] translations = new Translation2d[modules.size()];

        for(int i = 0; i < modules.size(); i++) translations[i] = modules.get(i).translation;
        
        kinematics = new SwerveDriveKinematics(translations);
    }

    public Rotation2d get_angle() {
        return Rotation2d.fromDegrees(imu.getAngle());
    }

    public void set(double x_vel, double y_vel, double a_vel) {
        speeds.vxMetersPerSecond = x_vel;
        speeds.vyMetersPerSecond = x_vel;
        speeds.omegaRadiansPerSecond = a_vel;
    }

    public void set(double x_vel, double y_vel) {
        set(x_vel, y_vel, 0);
    }

    public void set(double vel, Rotation2d heading) {
        set(vel * heading.getCos(), vel * heading.getSin());
    }

    public void set(double vel, Rotation2d heading, double a_vel) {
        set(vel * heading.getCos(), vel * heading.getSin(), a_vel);
    }

    @Override
    public void periodic() {
        // Set Module Speeds
        var speeds_fr = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, get_angle());
        var states = kinematics.toSwerveModuleStates(speeds_fr);

        // TODO Offset angle rate to compensate for error from target angle rate

        for(int i = 0; i < Math.min(modules.size(), states.length); i++) {
            modules.get(i).set_target(states[i]);
        }
    }

    // Static Initializer
    public static Drivetrain create() {
        if (inst == null) {
            inst = new Drivetrain();
        }

        return inst;
    }

}
