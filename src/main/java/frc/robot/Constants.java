package frc.robot;

public class Constants {
    class PIDParam{
        double kP;
        double kI;
        double kD;

        public PIDParam(double kP, double kI, double kD){
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    class SimpleFFParam{
        double kS;
        double kV;
        double kA;

        public SimpleFFParam(double kS, double kV) {
            this.kS = kS;
            this.kV = kV;
            this.kA = 0;

        }

        public SimpleFFParam(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;

        }
    }
}
