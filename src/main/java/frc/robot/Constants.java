package frc.robot;

import frc.robot.util.PosController;
import frc.robot.util.RateController;

public class Constants {
    class MotorSpecs {
        public final double fs;
        /** < Free speed - RPM */
        public final double fc;
        /** < Free current - A */
        public final double st;
        /** < Stall Torque - N-m */
        public final double sc;
        /** < Stall Current - A */

        public final double volt_nom;

        /** < Nominal Voltage - V */

        /**
         * Constructor
         * 
         * @param fs Free Speed in RPM
         * @param fc Free Current in Amps
         * @param st Stall Torque in N-m
         * @param sc Stall Current in Amps
         */
        public MotorSpecs(double fs, double fc, double st, double sc) {
            this.fs = fs;
            this.fc = fc;
            this.st = st;
            this.sc = sc;
            volt_nom = 12;
        }

        /**
         * Constructor
         * 
         * @param fs       Free Speed in RPM
         * @param fc       Free Current in Amps
         * @param st       Stall Torque in N-m
         * @param sc       Stall Current in Amps
         * @param volt_nom Nominal Voltage in Volts
         */
        public MotorSpecs(double fs, double fc, double st, double sc, double volt_nom) {
            this.fs = fs;
            this.fc = fc;
            this.st = st;
            this.sc = sc;
            this.volt_nom = volt_nom;
        }

        /**
         * Calculate Speed Constant
         * 
         * @return Speed Constant
         */
        public double speed_const() {
            return fs / volt_nom;
        }
    }

    public MotorSpecs f500 = new MotorSpecs(6380, 1.5, 4.69, 257);

    public double drive_ratio = 1 / 6.75;
    public double drive_angle_ratio = 1 / (150 / 7);

    public double drive_wheel_circ = Math.PI * 4;

    public RateController.FFParam drive_ff = new RateController.FFParam(1.0,
            f500.speed_const() / 60 * drive_ratio * drive_wheel_circ);
    public RateController.PIDParam drive_angle_pid = new RateController.PIDParam(.0001, 0, 0);
    public RateController.FFParam drive_angle_ff = new RateController.FFParam(1,
            f500.speed_const() / 60 * drive_angle_ratio);

    public PosController.PosParam angle_pos_param = new PosController.PosParam(2 * Math.PI, 2 * Math.PI / .05);

    public double robot_dims = 29.5 / .0254;
    public double module_inset = 2.5 / .0254;
    public double module_offset = robot_dims / 2 - module_inset;

    public double max_drive_speed = f500.speed_const() / 60 * 12;
    public double max_angle_rate = 360 * 5;

    public static Constants inst;

    public static Constants get_instance() {
        if (inst == null) {
            inst = new Constants();
        }

        return inst;
    }
}
