package org.firstinspires.ftc.teamcode;

/**
 *  Move Profile object.
 *  Methods to drive specified distance in specified time using curves to smooth the motion.
 *  Position, velocity curves are used.
 */
public class MoveProfiles {
    // members
    public boolean reverseDir = false; // for running backwards
    
    private final PiecewiseFunction posCurve = new PiecewiseFunction();
    private final PiecewiseFunction veloCurve = new PiecewiseFunction();

    /**
     * Constructor.  Initializes the position curve.
     * @param time seconds
     * @param distance how far to move in mm
      */
    public MoveProfiles(double time, double distance) {

        posCurve.debug = false;
        fillSineWave(20,time,distance,(- Math.PI/2.0), Math.PI, true, posCurve);

        fillDerivative(posCurve,veloCurve);
        veloCurve.debug = false;

    }

    /**
     * IN-WORK. Code not complete
     * @param dist  distance
     * @param vMax  velocity maximum
     * @param aMax  acceleration maximum
     */
    public void makeTrapizoidMotionProfile(double dist,double vMax, double aMax) {
        // Check if the triangular profile is needed (cannot reach max velocity)
        double accelDist = (vMax * vMax) / aMax;

        if (accelDist > dist) {
            // Triangular profile
            vMax = Math.sqrt(dist * aMax);
        }

        double tAccel = vMax / aMax;
        double dAccel = 0.5 * aMax * tAccel * tAccel;
        double dCruise = dist - (2.0 * dAccel);
        double tCruise = dCruise / vMax;
        double tTotal = (2.0 * tAccel) + tCruise;

    }

    /**
     * lineMoveLoop method, to be called continuously for the duration of the requested move.
     * Scales the position and pitch curves to the requested values.
     * @param currentTime (seconds) from start of move
     * @param startingS (mm) starting position S
     * @return array [3] containing posTarget and velocityTarget
     */
    public double[] lineMoveLoop(double currentTime, double startingS) {
        double posTarget;
        double velocity;
        int sign; // for direction

        if (reverseDir) sign = -1;
        else sign = 1;

        posTarget = sign* posCurve.getY(currentTime) + startingS;

        velocity = sign* veloCurve.getY(currentTime);

        return new double[] {posTarget, velocity};
    }

    /**
     * Builds a sine curve from -PI/2 to PI/2 (180 degrees)
     * @param pieces number of pieces in the curve, from 2 to 100
     * @param time how long the move should take in seconds.  Starts at zero.
     * @param amplitude The y axis value at PI/2.  Starts at zero
     * @param period the periodic length of the curve, in radians
     * @param phase phase shift of the sine function, in radians
     * @param curve The sine curve as a piecewiseFunction
     */
    public void fillSineWave(int pieces, double time, double amplitude, double phase,
                             double period, boolean shift,PiecewiseFunction curve ) {

        double Dist;

        if (pieces < 3) pieces = 2;
        if (pieces > 100) pieces = 100;
        if (time < 0) time -= time;
        if (time == 0) time = 1.0;

        for (int i=0; i <= pieces; i++)  {
            double iTime =  ((double)i /(double)pieces) * time;
            double angle =  ((double)i /(double)pieces) * period + phase;
            if (shift) Dist = (amplitude/2) * (Math.sin((angle))+1.0 );
            else Dist = amplitude * Math.sin(angle);
            curve.addElement( iTime,    Dist);
        }
    }

    /**
     * Builds a derivative PiecewiseFunction from a given curve, forces both ends to zero slope.
     * Assumes a uniform x (time) step
     * @param curve PiecewiseFunction
     * @param derivative PiecewiseFunction
     */
    public void fillDerivative(PiecewiseFunction curve, PiecewiseFunction derivative) {
        int pieces = curve.getSize()-1; // getSize returns points, we need pieces

        derivative.addElement(0.0,0.0);
        double slope1 = (curve.getElementY(0)- curve.getElementY(1))/
                (curve.getElementX(0)- curve.getElementX(1));

        for (int i=1; i < pieces; i++) {
            double slope2 = (curve.getElementY(i)- curve.getElementY(i+1))/
                    (curve.getElementX(i)- curve.getElementX(i+1));
            derivative.addElement(curve.getElementX(i),(slope1+slope2)/2.0);
            slope1=slope2;
        }
        derivative.addElement(curve.getElementX(pieces),0.0);
    }

    /**
     * Builds an Integral PiecewiseFunction from a given curve. Starts at zero.
     *  Assumes a uniform x (time) step
     * @param curve PiecewiseFunction
     * @param integral PiecewiseFunction
     */
    public void fillIntegral(PiecewiseFunction curve, PiecewiseFunction integral) {
        int pieces = curve.getSize()-1; // getSize returns points, we need pieces
        double tStep = curve.getElementX(1)- curve.getElementX(0);

        integral.addElement(0.0,0.0);
        double area = ((curve.getElementY(1)+
                curve.getElementY(0))/2.0) * tStep;

        for (int i=1; i <= pieces; i++) {
            double nextArea = ((curve.getElementY(i+1)+ curve.getElementY(i))/2.0)/
                    tStep;
            area = area + nextArea;
            integral.addElement(curve.getElementX(i),area);
        }
    }
    /**
     * Returns the maximum velocity for a robot starting at rest, following a sine profile,
     * and stopping at the end.
     * @param distance = distance traveled
     * @param time = time to travel the distance
     * @return = the maximum velocity reached during the travel
     */
    //public double getMaxVelocity(double distance, double time) {return 2.0*distance / time;  }
}
