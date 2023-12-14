package frc.robot.util;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;



public class RateController {
    public class PIDParam{
        public double kP;    /**< Proportional Gain Constant */
        public double kI;    /**< Intergral Gain Constant */
        public double kD;    /**< Differential Gain Constant */

        public PIDParam(float kP, float kI, float kD){
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public class FFParam {        
        public enum FFMode { SIMPLE, ARM, ELEVATOR }
        
        public double kS;    /**< Dead Zone Voltage (volts) */ 
        public double kG;    /**< Gravity Offet Voltage (volts) */
        public double kV;    /**< Velocity Voltage Contant  (v * s / distance for simple and elevator, v * s / radians for arm) */
        public double kA;    /**< Accleration Voltage Constant (v * s ^ 2 / distance  for simple and elevator, v * s ^ 2 / radians for arm) */

        public FFMode mode; /**< Feed Forward Mode */
        
        public FFParam(float kS, float kV) {
            this.kS = kS;
            this.kV = kV;
            this.kA = 0;
            this.kG = 0;
            this.mode = FFMode.Simple;
        }

        public FFParam(float kS, float kV, float kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = 0;
            this.mode = FFMode.Simple;
        }

        public FFParam(float kS, float kV float kG, FFMode mode) {
            this.kS = kS;
            this.kV = kV;
            this.kA = 0;
            this.kG = kG;
            this.mode = mode;
        }

        public FFParam(float kS, float kV float kG, float kA, FFMode mode) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
            this.mode = mode;
        }
        
    }

    private PIDController pid_ctrl;
    private SimpleMotorFeedforward simple_ff;
    private ArmFeedforward arm_ff;
    private ElevatorFeedforward elevator_ff;


    public RateController(PIDParam pid_param){
        init_pid(pid_param);
        clear_ff();
    }


    public RateController(FFParam ff_param){
        clear_pid();
        init_ff(ff_param);
    }


    public RateController(PIDParam pid_param, FFParam ff_param){
        init_pid(pid_param);
        init_ff(ff_param);
    }

    public double calculate(ControlState current, ControlState target) {
        double result = 0;

        if(simple_ff != null) {
            result += simple_ff.calculate(target.vel, target.accel)
        } else if(arm_ff != null) {
            result += arm_ff.calculate(current.pos, target.vel, target.accel)
        } else if(elevator_ff != null) {
            result += elevator_ff.calculate(target.vel, target.accel)
        }

        if (pid_ctrl != null) {
            result += pid_ctrl.calculate(current.vel, target.vel);
        }

        return result;
    }   

    private void init_pid(PIDParam pid_param) {
        pid_ctrl = PIDController(pid_param.kP, pid_param.kI, pid_param.kD);
    }

    private void init_ff(FFParam ff_param) {
        clear_ff();

        switch(ff_param.mode) {
            case FFParam.FFMode.SIMPLE:
                simple_ff = SimpleMotorFeedforward(ff_param.kS, ff_param.kV, ff_param.kA);
                break;
            case FFParam.FFMode.ARM:
                arm_ff = ArmFeedforward(ff_param.kS, ff_param.kG, ff_param.kV, ff_param.kA);
                break;
            case FFParam.FFMode.SIMPLE:
                elevator_ff = ElevatorFeedforward(ff_param.kS, ff_param.kG, ff_param.kV, ff_param.kA);
                break;
        }
    }

    private void clear_pid() {
        pid_ctrl = null;
    }

    private void clear_ff() {
        simple_ff = null;
        arm_ff = null;
        elevator_ff = null;
    }
}