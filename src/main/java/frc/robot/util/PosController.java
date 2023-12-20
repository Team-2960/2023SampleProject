package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class PosController {
    public static class PosParam {
        public double max_rate;
        public double ramp_down;
        public double max_accel;

        public PosParam(double max_rate, double ramp_down) {
            this.max_rate = max_rate;
            this.ramp_down = ramp_down;
            this.max_accel = Double.POSITIVE_INFINITY;
        }

        public PosParam(double max_rate, double ramp_down, double max_accel) {
            this.max_rate = max_rate;
            this.ramp_down = ramp_down;
            this.max_accel = max_accel;
        }
    }

    private PosParam param;
    private double last_time;

    public PosController(PosParam param) {
        this.param = param;

        this.last_time = Timer.getFPGATimestamp();
    }

    public double calculate(ControlState current, ControlState target) {
        double cur_time = Timer.getFPGATimestamp();
        double result = calculate(current, target, cur_time - last_time);

        last_time = cur_time;

        return result;
    }

    public double calculate(ControlState current, ControlState target, double time_delta) {
        double error = target.pos - current.pos;

        double ramp_down = error * param.ramp_down;
        double ramp_up = (error > 0 ? 1 : -1) * param.max_accel + current.vel;
        double max_speed = (error > 0 ? 1 : -1) * param.max_rate;

        double result = max_speed;

        if (Math.abs(result) > Math.abs(ramp_down))
            result = ramp_down;
        if (Math.abs(result) > Math.abs(ramp_up))
            result = ramp_up;

        return result;
    }
}