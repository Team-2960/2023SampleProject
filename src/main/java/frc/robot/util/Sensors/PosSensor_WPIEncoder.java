package frc.robot.util.Sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class PosSensor_WPIEncoder extends PosSensor {
    private Encoder quad_enc;
    private DutyCycleEncoder dc_enc;

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     */
    public PosSensor_WPIEncoder(Encoder quad_enc) {
        this.quad_enc = quad_enc;
        this.dc_enc = null;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param dc_enc Duty Cycle Encoder
     */
    public PosSensor_WPIEncoder(DutyCycleEncoder dc_enc) {
        this.quad_enc = null;
        this.dc_enc = dc_enc;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param dc_enc   Duty Cycle Encoder
     */
    public PosSensor_WPIEncoder(Encoder quad_enc, DutyCycleEncoder dc_enc) {
        this.quad_enc = quad_enc;
        this.dc_enc = dc_enc;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param offset   Initial position Offset
     */
    public PosSensor_WPIEncoder(Encoder quad_enc, double offset) {
        super(offset);
        this.quad_enc = quad_enc;
        this.dc_enc = null;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param dc_enc Duty Cycle Encoder
     * @param offset Initial position Offset
     */
    public PosSensor_WPIEncoder(DutyCycleEncoder dc_enc, double offset) {
        this.quad_enc = null;
        this.dc_enc = dc_enc;
        set_offset(offset);
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param dc_enc   Duty Cycle Encoder
     * @param offset   Initial position Offset
     */
    public PosSensor_WPIEncoder(Encoder quad_enc, DutyCycleEncoder dc_enc, double offset) {
        this.quad_enc = quad_enc;
        this.dc_enc = dc_enc;
        set_offset(offset);
    }

    /**
     * Get the position of the sensor with the offset applied
     * 
     * @return position of the sensor with the offset applied
     */
    @Override
    public double get_position() {
        if (dc_enc == null) {
            return super.get_position();
        } else {
            return get_sensor_pos();
        }
    }

    /**
     * Sets the current offset to the current sensor position
     */
    @Override
    public void zero() {
        if (dc_enc == null) {
            quad_enc.reset();
            set_offset(0);
        } else {
            dc_enc.setPositionOffset(0);
            double pos = dc_enc.getAbsolutePosition();
            dc_enc.setPositionOffset(pos);
        }
    }

    /**
     * Sets the position offset
     * 
     * @param offset position offset
     */
    @Override
    public void set_offset(double offset) {
        if (dc_enc == null) {
            super.set_offset(offset);
        } else {
            dc_enc.setPositionOffset(offset);
        }
    }

    /**
     * Gets the position from the sensor
     * 
     * @return position from the sensor
     */
    @Override
    public double get_sensor_pos() {
        if (dc_enc == null) {
            return quad_enc.getDistance();
        } else {
            return dc_enc.getAbsolutePosition();
        }
    }

    /**
     * Gets the rate from the sensor
     * 
     * @return rate from the sensor
     */
    @Override
    public double get_rate() {
        if (quad_enc == null) {
            return 0;
        } else {
            return quad_enc.getRate();
        }
    }
}
