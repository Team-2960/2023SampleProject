package frc.robot.util.Sensors;

public abstract class PosSensor {
    protected double offset;

    /**
     * Default Constrcutor. Offset set to 0
     */
    public PosSensor() {
        offset = 0;
    }

    /**
     * Constructor
     * 
     * @param offset sensor offset
     */
    public PosSensor(double offset) {
        this.offset = offset;
    }

    /**
     * Get the position of the sensor with the offset applied
     * 
     * @return position of the sensor with the offset applied
     */
    public double get_position() {
        return get_sensor_pos() - offset;
    }

    /**
     * Sets the current offset to the current sensor position
     */
    public void zero() {
        this.offset = get_sensor_pos();
    }

    /**
     * Sets the position offset
     * 
     * @param offset position offset
     */
    public void set_offset(double offset) {
        this.offset = offset;
    }

    /**
     * Gets the current sensor position
     * 
     * @return current sensor position
     */
    public abstract double get_sensor_pos();

    /**
     * Gets the current sensor rate
     * 
     * @return current sensor rate
     */
    public abstract double get_rate();

}
