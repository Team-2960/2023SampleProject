package frc.robot.util.Sensors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class PosSensor_CTRETalonFX extends PosSensor {
    private WPI_TalonFX sensor;

    /**
     * Position sensor wrapper for CTRE Talon FX
     * 
     * @param sensor Position sensor
     */
    public PosSensor_CTRETalonFX(WPI_TalonFX sensor) {
        this.sensor = sensor;
    }

    /**
     * Position sensor wrapper for CTRE Talon FX
     * 
     * @param sensor Position sensor
     * @param offset Initial sensor offset
     */
    public PosSensor_CTRETalonFX(WPI_TalonFX sensor, double offset) {
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
        return sensor.getSelectedSensorPosition();
    }

    /**
     * Gets the rate from the sensor
     * 
     * @return rate from the sensor
     */
    @Override
    public double get_rate() {
        return sensor.getSelectedSensorVelocity();
    }
}
