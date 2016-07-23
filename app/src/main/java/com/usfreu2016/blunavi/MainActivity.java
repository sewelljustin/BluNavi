package com.usfreu2016.blunavi;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Environment;
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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity {

    final String TAG = "MainActivity";

    final int REQUEST_CODE_ASK_PERMISSIONS = 1;

    final long TIMER_DELAY = 500;

    final double VERT_ACC_THRESHOLD = 2.0;
    final double HORI_ACC_THRESHOLD = 1.0;
    final long STEP_TIME_THRESHOLD = 500;

    final double BEARING_THRESHOLD = 10;

    final String dirName = "BluNavi Data";


    /** Timer and task for recording position */
    Timer timer;
    TimerTask task;

    /** Eddystone scanning references*/
    BeaconManager beaconManager;
    String scanId;
    boolean scanning;
    boolean beaconManagerServiceReady;

    /** Dead reckoning references */
    DeadReckoningManager deadReckoningManager;
    int bearing;
    double radBearing; // bearing expressed in radians
    boolean deadReckoningManagerServiceReady;
    double previousStepLength;

    /** Kalman Filter references */
    final double PROCESS_NOISE_VARIANCE = 0.0064;
    final double MEASUREMENT_NOISE_VARIANCE = 16.8921;
    final double STATE_VARIANCE = 500.0;
    ExtendedKalmanFilter filter;

    /** Used to write the two files */
    BufferedWriter positionWriter;
    BufferedWriter timeWriter;

    /** Estimated position */
    double xHat;
    double yHat;

    /** Views */
    Button startButton;
    Button setInitialOrientationButton;
    Button recordTimeButton;
    TextView dataTextView;

    boolean recordingData;

    Eddystone focusedBeacon;

    //nsk4UG
    BluNaviBeacon beacon1 = new BluNaviBeacon("nsk4UG", "FF:48:85:91:B0:0D", 0, -1.22);

    //J8Afaf
    BluNaviBeacon beacon2 = new BluNaviBeacon("J8Afaf", "D5:00:25:D5:22:A9", 0.61, 7.32);

    //4L6DDN
    BluNaviBeacon beacon3 = new BluNaviBeacon("4L6DDN", "CB:37:96:6C:06:E2", 0, 17.07);

    //zPtPxR
    BluNaviBeacon beacon4 = new BluNaviBeacon("zPtPxR", "D1:07:0C:8F:45:90", -13.41, -1.22);

    //kNYXCP
    BluNaviBeacon beacon5 = new BluNaviBeacon("kNYXCP", "CD:13:1D:A6:C4:2E", -26.82, -1.22);

    //UDbczx
    BluNaviBeacon beacon6 = new BluNaviBeacon("UDbczx", "F3:EE:D0:01:5B:AE", -13.41, 17.07);

    //5tXSCU
    BluNaviBeacon beacon7 = new BluNaviBeacon("5tXSCU", "E7:4E:95:C8:62:A3", -27.43, 16.46);

    //S3aP63
    BluNaviBeacon beacon8 = new BluNaviBeacon("S3aP63", "DE:38:78:85:1C:6D", -27.43, 7.32);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        timer = new Timer();
        xHat = 0;
        yHat = 0;
        previousStepLength = 0.75;
        recordingData = false;
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
    }

    private boolean validBearing(int bearing) {
        if (Math.abs(this.bearing - bearing) >= BEARING_THRESHOLD) {
            return true;
        }
        return false;
    }

    private void checkFocusedBeacon(List<Eddystone> list) {

        // create list of beacons that are in view of the device
        List<Eddystone> validOptions = new ArrayList<>();
        for (Eddystone eddystone : list) {

            if (eddystone.macAddress.equals(beacon1.getMacAddress())) {
                double bY = beacon1.getCoords()[1];
                double bX = beacon1.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon2.getMacAddress())) {
                double bY = beacon2.getCoords()[1];
                double bX = beacon2.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon3.getMacAddress())) {
                double bY = beacon3.getCoords()[1];
                double bX = beacon3.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon4.getMacAddress())) {
                double bY = beacon4.getCoords()[1];
                double bX = beacon4.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon5.getMacAddress())) {
                double bY = beacon5.getCoords()[1];
                double bX = beacon5.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon6.getMacAddress())) {
                double bY = beacon6.getCoords()[1];
                double bX = beacon6.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon7.getMacAddress())) {
                double bY = beacon7.getCoords()[1];
                double bX = beacon7.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
            else if (eddystone.macAddress.equals(beacon8.getMacAddress())) {
                double bY = beacon8.getCoords()[1];
                double bX = beacon8.getCoords()[0];
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
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
        recordTimeButton = (Button) findViewById(R.id.recordTimeButton);
        dataTextView = (TextView) findViewById(R.id.dataTextView);

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
                if (startButton.getText().toString().equals("Start")) {
                    if (beaconManagerServiceReady && deadReckoningManagerServiceReady) {

                        /** Initial Position */
                        xHat = 0;
                        yHat = 0;
                        buildFilter();

                        openFileWriters();
                        task = new TimerTask() {
                            @Override
                            public void run() {
                                if (recordingData) {
                                    recordPosition();
                                }
                            }
                        };
                        timer.schedule(task, 0, TIMER_DELAY);

                        beaconManager.startEddystoneScanning();
                        deadReckoningManager.startStepDetection();
                        deadReckoningManager.setVertAccThreshold(VERT_ACC_THRESHOLD);
                        deadReckoningManager.setHoriAccThreshold(HORI_ACC_THRESHOLD);
                        deadReckoningManager.setStepTimeThreshold(STEP_TIME_THRESHOLD);
                        deadReckoningManager.startOrientationTracking();
                        startButton.setText("Stop");
                    } else {
                        Toast.makeText(MainActivity.this, "Try again.", Toast.LENGTH_SHORT).show();
                    }
                }
                else {
                    closeFileWriters();
                    task.cancel();
                    startButton.setText("Start");
                    recordingData = false;
                }
            }
        });
        setInitialOrientationButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                recordingData = true;
                deadReckoningManager.setInitialOrientation();
            }
        });
        recordTimeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                recordTime();
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
            }
        });
        deadReckoningManager.setOrientationListener(new DeadReckoningManager.OrientationListener() {
            @Override
            public void onChanged(int angle) {
                bearing = angle;
                radBearing = (Math.PI * bearing) / 180;
                filter.setRadBearing(radBearing);
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

    private void recordPosition() {
        if (recordingData) {
            long time = Calendar.getInstance().getTimeInMillis();
            final String data = String.format("%.2f,%.2f,%d,%d", xHat, yHat, bearing, time);
            try {
                positionWriter.append(data);
                positionWriter.append("\n");
            } catch (FileNotFoundException e) {
                Log.e("Error", "File not found exception");
            } catch (IOException e) {
                Log.e("Error", "IOException");
            }
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    dataTextView.setText(data);
                }
            });
        }
    }

    private void recordTime() {
        if (recordingData) {
            try {
                timeWriter.append(String.valueOf(Calendar.getInstance().getTimeInMillis()));
                timeWriter.append("\n");
            } catch (FileNotFoundException e) {
                Log.e("Error", "File not found exception");
            } catch (IOException e) {
                Log.e("Error", "IOException");
            }
        }
    }

    private void openFileWriters() {
        /** Check for write permissions */
        checkPermissions();

        /** Create the files */
        File dir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), dirName);
        if (!dir.exists()) {
            if (!dir.mkdirs()) {
                Log.e("Error", "Directory not created.");
            }
        }
        File positionFile = new File(dir, "position.csv");
        File timeFile = new File(dir, "time.csv");

        /** Set up the writers */
        try {
            FileWriter writer1 = new FileWriter(positionFile, true);
            FileWriter writer2 = new FileWriter(timeFile, true);
            positionWriter = new BufferedWriter(writer1);
            timeWriter = new BufferedWriter(writer2);
        } catch (FileNotFoundException e) {
            Log.e("Error", "File not found exception");
        } catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    private void closeFileWriters() {
        /** close the writers */
        try {
            positionWriter.close();
            timeWriter.close();
        } catch (FileNotFoundException e) {
            Log.e("Error", "File not found exception");
        } catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    private void checkPermissions() {
        int hasWriteExternalStoragePermission = checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (hasWriteExternalStoragePermission != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    REQUEST_CODE_ASK_PERMISSIONS);
            return;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        switch (requestCode) {
            case REQUEST_CODE_ASK_PERMISSIONS:
                if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                }
                else {
                    Toast.makeText(this, "Permissions Denied", Toast.LENGTH_SHORT).show();
                }
                break;
            default:
                super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }

}
