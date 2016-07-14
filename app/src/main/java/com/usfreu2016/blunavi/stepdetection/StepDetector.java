package com.usfreu2016.blunavi.stepdetection;


import java.util.Calendar;

public class StepDetector {

    private StepDetectedListener stepDetectedListener;
    private double vertAccThreshold;
    private double stepLength;
    private double stepLengthConstant;
    private double peakVertAcc;
    private double valleyVertAcc;
    private boolean stepDetected;
    private boolean peakVertAccRec;
    private boolean startValleyVertAccRec;
    private boolean valleyVertAccRec;

    public StepDetector() {
        this.vertAccThreshold = 2;
        this.stepLengthConstant = 0.53;
        this.peakVertAcc = 0;
        this.valleyVertAcc = 0;
        this.stepDetected = false;
        this.peakVertAccRec = false;
        this.startValleyVertAccRec = false;
        this.valleyVertAccRec = false;
    }

    /**
     * Add values from a linear acceleration sensor
     *
     * @param values x, y, z values of the linear acceleration virtual sensor
     */
    public void add(float[] values) {
        checkVerticalValue(values[1]);
    }

    /**
     * Check if vertical value has completed a step cycle
     * @param value The vertical acceleration value
     */
    private void checkVerticalValue(double value) {

        /** if peak and valley acceleration are recorded and the current value
         * is greater greater than threshold, then step detected
         */
        if (this.peakVertAccRec && this.valleyVertAccRec && (value > this.vertAccThreshold)) {
            /** Compute step length */
            this.stepLength = computeStepLength(this.peakVertAcc, this.valleyVertAcc);

            /** Fire step detected event chain */
            this.stepDetectedListener.onStepDetected(
                    new Step(this.stepLength, Calendar.getInstance().getTimeInMillis()));

            /** Reset for next step */
            this.peakVertAcc = 0;
            this.valleyVertAcc = 0;
            this.peakVertAccRec = false;
            this.startValleyVertAccRec = false;
            this.valleyVertAccRec = false;
        }

        /** If value is greater than the vertical acceleration threshold */
        if (value > this.vertAccThreshold) {

            // record peak acceleration
            if (value > this.peakVertAcc) {
                this.peakVertAcc = value;
                this.peakVertAccRec = true;
            }
        }

        /** if peak is recorded and acceleration is decreasing */
        if (this.peakVertAccRec && (value - this.peakVertAcc) < 0) {
            startValleyVertAccRec = true;
        }

        /** if currently recording valley of vertical acceleration */
        if (startValleyVertAccRec && value < valleyVertAcc) {
            valleyVertAcc = value;
            valleyVertAccRec = true;
        }
    }

    /**
     * Set the threshold for vertical acceleration step detection
     * @param threshold The threshold value
     */
    public void setVertAccThreshold(double threshold) {
        this.vertAccThreshold = threshold;
    }

    /**
     * Set a listener for step detection
     * @param stepDetectedListener
     */
    public void setStepDetectedListener(StepDetectedListener stepDetectedListener) {
        this.stepDetectedListener = stepDetectedListener;
    }

    /**
     * Interface for onStepDetected callback
     */
    public interface StepDetectedListener {
        void onStepDetected(Step step);
    }

    private double computeStepLength(double max, double min) {
        double diff = max - min;
        double result = stepLengthConstant * (float) Math.pow(diff, 1.0 / 4);
        return result;
    }
}
