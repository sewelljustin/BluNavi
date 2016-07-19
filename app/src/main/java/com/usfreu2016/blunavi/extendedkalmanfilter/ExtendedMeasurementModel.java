package com.usfreu2016.blunavi.extendedkalmanfilter;


import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class ExtendedMeasurementModel extends DefaultMeasurementModel {

    private double radBearing;

    public ExtendedMeasurementModel(RealMatrix measMatrix, RealMatrix measNoise) {
        super (measMatrix, measNoise);
    }

    @Override
    public RealMatrix getMeasurementNoise() {
        double[][] measurementNoiseArray = super.getMeasurementNoise().getData();
        measurementNoiseArray[0][0] = measurementNoiseArray[0][0] * (Math.abs((int) Math.sin(radBearing)));
        measurementNoiseArray[1][1] = measurementNoiseArray[1][1] * (Math.abs((int) Math.cos(radBearing)));
        RealMatrix MeasurementNoiseMatrix = new Array2DRowRealMatrix(measurementNoiseArray);
        return MeasurementNoiseMatrix;
    }

    public void setRadBearing(double radBearing) {
        this.radBearing = radBearing;
    }
}
