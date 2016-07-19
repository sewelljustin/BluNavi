package com.usfreu2016.blunavi.extendedkalmanfilter;


import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class ExtendedProcessModel extends DefaultProcessModel {

    private double radBearing;

    public ExtendedProcessModel(RealMatrix stateTransition, RealMatrix control, RealMatrix processNoise, RealVector initialStateEstimate, RealMatrix initialErrorCovariance) {
        super(stateTransition, control, processNoise, initialStateEstimate, initialErrorCovariance);
        this.radBearing = 0;
    }

    @Override
    public RealMatrix getProcessNoise() {
        double[][] processNoiseArray = super.getProcessNoise().getData();

        // system will have very little noise in the direction that is not being traversed
        // this accounts for that
        processNoiseArray[0][0] = processNoiseArray[0][0] * (Math.abs((int) Math.sin(radBearing)));
        processNoiseArray[1][1] = processNoiseArray[1][1] * (Math.abs((int) Math.cos(radBearing)));

        // There must still be some noise for the Kalman Filter's methods to work
        if (processNoiseArray[0][0] == 0) {
            processNoiseArray[0][0] += 1.0e-9;
        }
        if (processNoiseArray[1][1] == 0) {
            processNoiseArray[1][1] += 1.0e-9;
        }


        RealMatrix processNoiseMatrix = new Array2DRowRealMatrix(processNoiseArray);
        return processNoiseMatrix;
    }

    public void setRadBearing(double radBearing) {
        this.radBearing = radBearing;
    }
}
