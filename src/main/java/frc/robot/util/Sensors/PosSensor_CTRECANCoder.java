package frc.robot.util.Sensors;

import com.ctre.phoenix.sensors.CANCoder;

public class PosSensor_CTRECANCoder extends PosSensor {
    private CANCoder sensor;

    /**
     * Position sensor wrapper for CTRE CANCoder
     * 
     * @param sensor Position sensor
     */
    public PosSensor_CTRECANCoder(CANCoder sensor) {
        this.sensor = sensor;
    }

    /**
     * Position sensor wrapper for CTRE CANCoder
     * 
     * @param sensor Position sensor
     * @param offset Initial position offset
     */
    public PosSensor_CTRECANCoder(CANCoder sensor, double offset) {
        super(offset);
        this.sensor = sensor;
    }

    /**
     * Gets the position from the sensor
     * 
     * @return position from the sensor
     */
    @Override
    public double get_sensor_pos() {
        return sensor.getAbsolutePosition();
    }

    /**
     * Gets the rate from the sensor
     * 
     * @return rate from the sensor
     */
    @Override
    public double get_rate() {
        return sensor.getVelocity();
    }
}
