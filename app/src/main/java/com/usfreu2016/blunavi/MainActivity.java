package com.usfreu2016.blunavi;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.SystemRequirementsChecker;
import com.estimote.sdk.eddystone.Eddystone;
import com.estimote.sdk.repackaged.okhttp_v2_2_0.com.squareup.okhttp.internal.Util;
import com.usfreu2016.blunavi.deadreckoning.DeadReckoningManager;
import com.usfreu2016.blunavi.deadreckoning.Step;
import com.usfreu2016.blunavi.extendedkalmanfilter.ExtendedKalmanFilter;
import com.usfreu2016.blunavi.extendedkalmanfilter.ExtendedMeasurementModel;
import com.usfreu2016.blunavi.extendedkalmanfilter.ExtendedProcessModel;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MainActivity extends AppCompatActivity {

    final String TAG = "MainActivity";

    final double VERT_ACC_THRESHOLD = 2.0;
    final double HORI_ACC_THRESHOLD = 1.0;
    final long STEP_TIME_THRESHOLD = 500;


    /** Eddystone scanning references*/
    BeaconManager beaconManager;
    String scanId;
    boolean scanning;
    boolean beaconManagerServiceReady;

    /** Comparator for sorting Eddystones */
    final EddystoneComparator eddystoneComparator = new EddystoneComparator();

    /** Dead reckoning references */
    DeadReckoningManager deadReckoningManager;
    int numberOfSteps = 0;
    int bearing;
    double radBearing; // bearing expressed in radians
    boolean deadReckoningManagerServiceReady;
    double previousStepLength;

    /** Kalman Filter references */
    final double PROCESS_NOISE_VARIANCE = 0.0064;
    final double MEASUREMENT_NOISE_VARIANCE = 16.8921;
    final double STATE_VARIANCE = 500.0;
    ExtendedKalmanFilter filter;

    /** Estimated position */
    double xHat;
    double yHat;

    /** Views */
    Button startButton;
    Button setInitialOrientationButton;
    TextView stepsTextView;
    TextView stepLengthTextView;
    TextView orientationTextView;
    TextView positionTextView;
    TextView focusedBeaconTextView;

    /** BAD CODE WILL CHANGE LATER */
    Eddystone focusedBeacon;

    //J8Afaf (-10, 0)
    BluNaviBeacon beacon1 = new BluNaviBeacon("J8Afaf", "D5:00:25:D5:22:A9", -10, 0);

    //nsk4UG (0, 0)
    BluNaviBeacon beacon2 = new BluNaviBeacon("nsk4UG", "FF:48:85:91:B0:0D", 0, 0);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        xHat = 0;
        yHat = 0;
        previousStepLength = 0.75;
//        Log.d(TAG, "bearing: " + Utils.computeBearing(0, 0, 0, 0));
//        Log.d(TAG, "atan2: " + Math.atan2(0.05967668696, -0.00681261948) * 57.2958);
        init();
        buildFilter();
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (beaconManagerServiceReady && !scanning) {
            startScanning();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (scanning) {
            stopScanning();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        beaconManager.disconnect();
        deadReckoningManager.disconnect();
    }

    private void startScanning() {
        SystemRequirementsChecker.checkWithDefaultDialogs(this);
        scanId = beaconManager.startEddystoneScanning();
        scanning = true;
    }

    private void stopScanning() {
        beaconManager.stopEddystoneScanning(scanId);
        scanning = false;
    }

    private void updatePosition() {
        xHat = filter.getStateEstimation()[0];
        yHat = filter.getStateEstimation()[1];
        positionTextView.setText("X: " + xHat + " Y: " + yHat);
    }

    private void checkFocusedBeacon(List<Eddystone> list) {

        // create list of beacons that are in view of the device
        List<Eddystone> validOptions = new ArrayList<>();
        for (Eddystone eddystone : list) {

            if (eddystone.macAddress.equals(beacon1.getMacAddress())) {
                int bY = beacon1.getCoords()[1];
                int bX = beacon1.getCoords()[0];
                if ((Utils.computeBearing(yHat, xHat, bY, bX)) == bearing) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon2.getMacAddress())) {
                int bY = beacon2.getCoords()[1];
                int bX = beacon2.getCoords()[0];
                if ((Utils.computeBearing(yHat, xHat, bY, bX)) == bearing) {
                    validOptions.add(eddystone);
                }
            }
        }

        // if no valid options, then set focused beacon to closest beacon
        // list is sorted by rssi values
        if (validOptions.size() == 0) {
            focusedBeacon = list.get(0);
        }
        else {
            focusedBeacon = validOptions.get(0);
        }

        focusedBeaconTextView.setText(focusedBeacon.macAddress.toStandardString());

    }

    private RealVector getBeaconMeasurementVector() {
        double distance = Utils.computeBeaconDistance(focusedBeacon);
        double x = (((int) Math.sin(radBearing)) * -1) * distance + Math.abs((int) Math.cos(radBearing)) * xHat;
        double y = (((int) Math.cos(radBearing)) * -1) * distance + Math.abs((int) Math.sin(radBearing)) * yHat;
        RealVector z = new ArrayRealVector(new double[] {x, y});
        return z;
    }

    private RealVector getStepMeasurementVector(double stepLength) {
        double x = stepLength * ((int) Math.sin(radBearing));
        double y = stepLength * ((int) Math.cos(radBearing));
        RealVector z = new ArrayRealVector(new double[] {x, y});
        return z;

    }

    private void init() {

        /** Set up Views */
        startButton = (Button) findViewById(R.id.startButton);
        setInitialOrientationButton = (Button) findViewById(R.id.setInitialOrientationButton);
        stepsTextView = (TextView) findViewById(R.id.stepsTextView);
        stepsTextView.setText(String.valueOf(numberOfSteps));
        stepLengthTextView = (TextView) findViewById(R.id.stepLengthTextView);
        stepLengthTextView.setText(String.valueOf(0.0));
        orientationTextView = (TextView) findViewById(R.id.orientationTextView);
        orientationTextView.setText("0");
        positionTextView = (TextView) findViewById(R.id.positionTextView);
        positionTextView.setText("X: 0, Y: 0");
        focusedBeaconTextView = (TextView) findViewById(R.id.focusedBeaconTextView);

        /** Set up references for Eddystone scanning */
        beaconManager = new BeaconManager(getApplicationContext());
        beaconManager.setForegroundScanPeriod(500,0);
        // check permissions
        SystemRequirementsChecker.checkWithDefaultDialogs(this);

        /** Set up references for step detection */
        deadReckoningManager = new DeadReckoningManager(getApplicationContext());

        /** Set listeners */
        startButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (beaconManagerServiceReady && deadReckoningManagerServiceReady) {
                    beaconManager.startEddystoneScanning();
                    deadReckoningManager.startStepDetection();
                    deadReckoningManager.setVertAccThreshold(VERT_ACC_THRESHOLD);
                    deadReckoningManager.setHoriAccThreshold(HORI_ACC_THRESHOLD);
                    deadReckoningManager.setStepTimeThreshold(STEP_TIME_THRESHOLD);
                    deadReckoningManager.startOrientationTracking();
                }
                else {
                    Toast.makeText(MainActivity.this, "Try again.", Toast.LENGTH_SHORT).show();
                }
            }
        });
        setInitialOrientationButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                deadReckoningManager.setInitialOrientation();
            }
        });
        beaconManager.setEddystoneListener(new BeaconManager.EddystoneListener() {
            @Override
            public void onEddystonesFound(List<Eddystone> list) {
                if (list.size() > 0) {
                    checkFocusedBeacon(list);
                    filter.predict();
                    RealVector z = getBeaconMeasurementVector();
                    filter.correct(z);
                    updatePosition();
                }
            }
        });
        deadReckoningManager.setStepListener(new DeadReckoningManager.StepListener() {
            @Override
            public void onStep(Step step) {
                RealVector z = getStepMeasurementVector(previousStepLength);
                filter.predict(z);
                previousStepLength = step.getStepLength();
                z = getStepMeasurementVector(step.getStepLength());
                filter.correct(z);
                updatePosition();
                numberOfSteps++;
                stepsTextView.setText(String.valueOf(numberOfSteps));
                stepLengthTextView.setText(String.valueOf(step.getStepLength()));
            }
        });
        deadReckoningManager.setOrientationListener(new DeadReckoningManager.OrientationListener() {
            @Override
            public void onChanged(int angle) {
                bearing = angle;
                radBearing = (Math.PI * bearing) / 180;
                filter.setRadBearing(radBearing);
                orientationTextView.setText(String.valueOf(angle));
            }
        });

        /** Connect beaconManager to its service */
        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                beaconManagerServiceReady = true;
            }
        });

        /** Connect deadReckoningManager to its service */
        deadReckoningManager.connect(new DeadReckoningManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                deadReckoningManagerServiceReady = true;
            }
        });
    }

    /** Builds Kalman Filter */
    private void buildFilter() {
        /** Matrices and Vectors */

        // State Vector (1 dimensional for test purposes)
        // x = [ X ]
        //     [ Y ]
        // Initial state = [ 0, 0 ]^T
        RealVector x = new ArrayRealVector(new double[] {0, 0});

        // State Transition Matrix
        // A = [ 1 0 ]
        //     [ 0  1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[][] {{1, 0},{0, 1}});

        // Control Input Matrix
        // B = [ 1 0 ]
        //     [ 0 1 ]
        RealMatrix B = new Array2DRowRealMatrix(new double[][] {{1, 0},{0, 1}});

        // Measurement Matrix
        // H = [ 1 0 ]
        //     [ 0 1 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[][] {{1, 0},{0, 1}});

        // Process Noise Covariance Matrix
        // Q = [ PROCESS_NOISE_VARIANCE 0 ]
        //     [ 0 PROCESS_NOISE_VARIANCE ]
        RealMatrix Q = new Array2DRowRealMatrix(
                new double[][] {{PROCESS_NOISE_VARIANCE, 0}, {0, PROCESS_NOISE_VARIANCE}});

        // Measurement Noise Covariance Matrix
        // R = [ MEASUREMENT_NOISE_VARIANCE 0]
        //     [0 MEASUREMENT_NOISE_VARIANCE ]
        RealMatrix R = new Array2DRowRealMatrix(
                new double[][] {{MEASUREMENT_NOISE_VARIANCE, 0}, {0, MEASUREMENT_NOISE_VARIANCE}});

        // State Covariance Matrix (Error Covariance Matrix)
        // P = [ STATE_VARIANCE 0 ]
        //     [ 0 STATE_VARIANCE ]
        RealMatrix P = new Array2DRowRealMatrix(
                new double[][] {{STATE_VARIANCE, 0}, {0, STATE_VARIANCE}});;


        /** Build Process Model for Kalman Filter
         * Built using:
         * A - the state transition matrix,
         * B - the control input matrix - null in this case since there will be no control variable
         * Q - the process noise covariance matrix.
         * x - the state matrix
         * P - the state covariance matrix
         */
        ProcessModel process = new DefaultProcessModel(A, B, Q, x, P);

        /** Build Measurement Model for Kalman Filter
         * Built using:
         * H - the measurement matrix
         * R - the measurement noise covariance matrix
         */
        MeasurementModel measurement = new DefaultMeasurementModel(H, R);

        /** Build Kalman Filter */
        filter = new ExtendedKalmanFilter(process, measurement);
    }

}
