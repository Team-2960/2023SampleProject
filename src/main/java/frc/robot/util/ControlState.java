package frc.robot.util;

class ControlState{
    public double pos;      /**< State Position - Set to 0 if irrelevant */
    public double vel;      /**< State Velocity - Set to 0 if irrelevant */
    public double accel;    /**< State Acceleration - Set to 0 if irrelevant */
    
    public ControlState(double vel) {
        this.pos = 0;
        this.vel = vel;
        this.accel = 0;
    }

    public ControlState(double pos, double vel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = 0;
    }
    
    public ControlState(double pos, double vel, double accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
    }
}