package com.usfreu2016.blunavi;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.eddystone.Eddystone;
import com.usfreu2016.blunavi.stepdetection.Step;
import com.usfreu2016.blunavi.stepdetection.StepDetectionManager;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.List;

public class MainActivity extends AppCompatActivity {

    final double VERT_ACC_THRESHOLD = 2.0;
    final double HORI_ACC_THRESHOLD = 2.0;
    final long STEP_TIME_THRESHOLD = 500;

    /** Eddystone scanning references*/
    BeaconManager beaconManager;
    String scanId;
    boolean scanning;
    boolean beaconManagerServiceReady;

    /** Step detection and length estimation references */
    StepDetectionManager stepDetectionManager;
    int numberOfSteps = 0;
    boolean stepDetectionManagerServiceReady;

    /** Kalman Filter references */
    final double PROCESS_NOISE_VARIANCE = 0.0064;
    final double MEASUREMENT_NOISE_VARIANCE = 40;
    final double STATE_VARIANCE = 500.0;
    KalmanFilter filter;

    /** Views */
    Button startButton;
    TextView stepsTextView;
    TextView stepLengthTextView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        init();
        buildFilter();
    }

    private void init() {

        /** Set up Views */
        startButton = (Button) findViewById(R.id.startButton);
        stepsTextView = (TextView) findViewById(R.id.stepsTextView);
        stepsTextView.setText(String.valueOf(numberOfSteps));
        stepLengthTextView = (TextView) findViewById(R.id.stepLengthTextView);
        stepLengthTextView.setText(String.valueOf(0.0));

        /** Set up references for Eddystone scanning */
        beaconManager = new BeaconManager(getApplicationContext());
        beaconManager.setForegroundScanPeriod(500,0);

        /** Set up references for step detection */
        stepDetectionManager = new StepDetectionManager(getApplicationContext());

        /** Set listeners */
        startButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (beaconManagerServiceReady && stepDetectionManagerServiceReady) {
                    beaconManager.startEddystoneScanning();
                    stepDetectionManager.startStepDetection();
                    stepDetectionManager.setVertAccThreshold(VERT_ACC_THRESHOLD);
                    stepDetectionManager.setHoriAccThreshold(HORI_ACC_THRESHOLD);
                    stepDetectionManager.setStepTimeThreshold(STEP_TIME_THRESHOLD);
                }
                else {
                    Toast.makeText(MainActivity.this, "Try again.", Toast.LENGTH_SHORT).show();
                }
            }
        });
        beaconManager.setEddystoneListener(new BeaconManager.EddystoneListener() {
            @Override
            public void onEddystonesFound(List<Eddystone> list) {

            }
        });
        stepDetectionManager.setStepListener(new StepDetectionManager.StepListener() {
            @Override
            public void onStep(Step step) {
                numberOfSteps++;
                stepsTextView.setText(String.valueOf(numberOfSteps));
                stepLengthTextView.setText(String.valueOf(step.getStepLength()));
            }
        });

        /** Connect beaconManager to its service */
        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                beaconManagerServiceReady = true;
            }
        });

        /** Connect stepDetectionManager to its service */
        stepDetectionManager.connect(new StepDetectionManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                stepDetectionManagerServiceReady = true;
            }
        });
    }

    /** Builds Kalman Filter */
    private void buildFilter() {
        /** Matrices and Vectors */

        // State Vector (1 dimensional for test purposes)
        // x = [  X ]
        //     [ dX ]
        // Initial state = [ 15 0 ]^T
        RealVector x = new ArrayRealVector(new double[] {15, 0});

        // State Transition Matrix
        // delta T equals 1 seconds
        // A = [ 1 1 ]
        //     [0  1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[][] {{1, 1},{0, 1}});

        // Control Input Matrix
        // No control input
        // B = [ ]
        RealMatrix B = null;

        // Measurement Matrix
        // Currently, only observing changes in position
        // H = [ 1 0 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[][] {{1, 0}, {0, 0}});

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
        filter = new KalmanFilter(process, measurement);
    }
}
