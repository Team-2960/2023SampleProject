package frc.robot.util.Sensors;

import com.revrobotics.*;

public class PosSensor_RevSparkMax extends PosSensor {
    private RelativeEncoder rel_enc;
    private SparkMaxAbsoluteEncoder abs_enc;

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     */
    public PosSensor_RevSparkMax(SparkMaxRelativeEncoder rel_enc) {
        this.rel_enc = rel_enc;
        this.abs_enc = null;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param dc_enc Duty Cycle Encoder
     */
    public PosSensor_RevSparkMax(SparkMaxAbsoluteEncoder abs_enc) {
        this.rel_enc = null;
        this.abs_enc = abs_enc;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param dc_enc   Duty Cycle Encoder
     */
    public PosSensor_RevSparkMax(RelativeEncoder rel_enc, SparkMaxAbsoluteEncoder abs_enc) {
        this.rel_enc = rel_enc;
        this.abs_enc = abs_enc;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param offset   Initial position Offset
     */
    public PosSensor_RevSparkMax(RelativeEncoder rel_enc, double offset) {
        super(offset);
        this.rel_enc = rel_enc;
        this.abs_enc = null;
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param dc_enc Duty Cycle Encoder
     * @param offset Initial position Offset
     */
    public PosSensor_RevSparkMax(SparkMaxAbsoluteEncoder abs_enc, double offset) {
        this.rel_enc = null;
        this.abs_enc = abs_enc;
        set_offset(offset);
    }

    /**
     * Position sensor wrapper for WPI Encoder
     * 
     * @param quad_enc Quadrature Encoder
     * @param dc_enc   Duty Cycle Encoder
     * @param offset   Initial position Offset
     */
    public PosSensor_RevSparkMax(RelativeEncoder rel_enc, SparkMaxAbsoluteEncoder abs_enc, double offset) {
        this.rel_enc = rel_enc;
        this.abs_enc = abs_enc;
        set_offset(offset);
    }

    @Override
    public double get_position() {
        if (abs_enc == null) {
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
        if (abs_enc == null) {
            rel_enc.setPosition(0);
            set_offset(0);
        } else {
            abs_enc.setZeroOffset(0);
            double pos = abs_enc.getPosition();
            abs_enc.setZeroOffset(pos);
        }
    }

    /**
     * Sets the position offset
     * 
     * @param offset position offset
     */
    @Override
    public void set_offset(double offset) {
        if (abs_enc == null) {
            super.set_offset(offset);
        } else {
            abs_enc.setZeroOffset(offset);
        }
    }

    /**
     * Gets the position from the sensor
     * 
     * @return position from the sensor
     */
    @Override
    public double get_sensor_pos() {
        if (abs_enc == null) {
            return rel_enc.getPosition();
        } else {
            return abs_enc.getPosition();
        }
    }

    /**
     * Gets the rate from the sensor
     * 
     * @return rate from the sensor
     */
    @Override
    public double get_rate() {
        if (rel_enc == null) {
            return abs_enc.getVelocity();
        } else {
            return rel_enc.getVelocity();
        }
    }
}
