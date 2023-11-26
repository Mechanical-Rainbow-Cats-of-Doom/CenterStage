package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.geometry.Vector2d;

/**
 * This class is currently not being used
 */
public class Bezier {
    private Vector2d P0, P1, P2, P3, Pn, Pr, Ps, Pv, Pj, Pk, Pm;

    Bezier(Vector2d P0, Vector2d P1, Vector2d P2, Vector2d P3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
        this.Pn = P3.minus(P0).plus(P1.times(3)).minus(P2.times(3));
        this.Pr = P0.times(3).minus(P1.times(6)).plus(P2.times(3));
        this.Ps = P1.times(3).minus(P0.times(3));
        this.Pv = P0;
        this.Pj = Pn.times(3);
        this.Pk = Pr.times(2);
        this.Pm = Ps;
    }

    public double newtonRaphson(Vector2d targetPoint, double t0, double tolerance, int maxIterations) {

        double t = t0;

        for (int i = 0; i<maxIterations; i++) {
            Vector2d foundPoint = bezierCurve(t);
            Vector2d foundPointDerivative = bezierCurveDerivative(t);

            double distance = foundPoint.minus(targetPoint).magnitude();

            if (distance < tolerance) {
                return t;
            }

            t -= (foundPoint.getX() * foundPointDerivative.getX() + foundPoint.getY() * foundPointDerivative.getY()) /
                    (Math.pow(foundPointDerivative.getX(), 2) + Math.pow(foundPointDerivative.getY(), 2));
        }

        return -1;
    }

    private Vector2d bezierCurve(double t) {
        return Pn.times(Math.pow(t, 3)).plus(Pr.times(Math.pow(t,2))).plus(Ps.times(t)).plus(Pv);
    }

    private Vector2d bezierCurveDerivative(double t) {
        return Pj.times(Math.pow(t, 2)).plus(Pk.times(t)).plus(Pm);
    }



}
