package com.usfreu2016.blunavi;

import android.Manifest;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.SystemRequirementsChecker;
import com.estimote.sdk.eddystone.Eddystone;
import com.usfreu2016.blunavi.deadreckoning.DeadReckoningManager;
import com.usfreu2016.blunavi.deadreckoning.Step;
import com.usfreu2016.blunavi.extendedkalmanfilter.ExtendedKalmanFilter;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
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
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
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

    // HashMap of saved beacons
    HashMap<String /*Mac address*/, BluNaviBeacon> beaconHashMap;

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

        // update shared preferences
        buildBeaconMap();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (scanning) {
            stopScanning();
        }

        // update shared preferences

        SharedPreferences settings = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
        SharedPreferences.Editor editor = settings.edit();

        // add beacon to shared preferences file
        HashSet<String> beacon_mac_hashset = (HashSet<String>) settings.getStringSet(getString(R.string.beacon_mac_set), new HashSet<String>());

        for (String macAddress : beaconHashMap.keySet()) {
            final BluNaviBeacon beacon = beaconHashMap.get(macAddress);
            editor.putString(macAddress + "_id", beacon.getId());
            editor.putString(macAddress + "_mac", beacon.getMacAddress());
            editor.putFloat(macAddress + "_x", (float) beacon.getXPos());
            editor.putFloat(macAddress + "_y", (float) beacon.getYPos());
            beacon_mac_hashset.add(macAddress);
        }

        editor.putStringSet(getString(R.string.beacon_mac_set), beacon_mac_hashset);
        editor.commit();
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

            String macAddress = eddystone.macAddress.toStandardString();

            if (beaconHashMap.containsKey(macAddress)) {
                final BluNaviBeacon beacon = beaconHashMap.get(macAddress);
                double bY = beacon.getYPos();
                double bX = beacon.getXPos();

                // check bearing of beacon
                if (validBearing((Utils.computeBearing(yHat, xHat, bY, bX)))) {
                    validOptions.add(eddystone);
                }
            }
        }

        // if no valid options, then set focused beacon to closest beacon
        // list is sorted by rssi values
        focusedBeacon = (validOptions.size() == 0) ? list.get(0) : validOptions.get(0);

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

        /** Build beaconHashMap */
        beaconHashMap = new HashMap<>();

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
                    checkSharedPreferences(list);
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

    private void buildBeaconMap() {
        SharedPreferences settings = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
        HashSet<String> beacon_mac_hashset = (HashSet<String>) settings.getStringSet(getString(R.string.beacon_mac_set), new HashSet<String>());
        for (String mac : beacon_mac_hashset) {
            String beaconId = settings.getString(mac + "_id", "null");
            String beaconMac = settings.getString(mac + "_mac", "null");
            double beaconX = settings.getFloat(mac + "_x", 0);
            double beaconY = settings.getFloat(mac + "_y", 0);
            final BluNaviBeacon beacon = new BluNaviBeacon(beaconId, beaconMac, beaconX, beaconY);
            beaconHashMap.put(mac, beacon);
        }
    }

    // if new found beacon, add to shared preferences file
    private void checkSharedPreferences(List<Eddystone> list) {

        for (Eddystone eddystone : list) {

            String macAddress = eddystone.macAddress.toStandardString();

            // check if beacon is not in beaconHashMap
            if (!beaconHashMap.containsKey(macAddress)) {

                // Toast for "found new beacon"
                Toast toast = Toast.makeText(getApplicationContext(), "New Beacon Found", Toast.LENGTH_SHORT);
                toast.show();

                // create new beacon object
                String beacon_id = "null";
                String beacon_mac = macAddress;
                double beacon_x_pos = 0;
                double beacon_y_pos = 0;
                final BluNaviBeacon beacon = new BluNaviBeacon(beacon_id, beacon_mac, beacon_x_pos, beacon_y_pos);

                // add beacon to hashmap;
                beaconHashMap.put(beacon_mac, beacon);
            }
        }
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


    // Permissions for writing to file
    private void checkPermissions() {
        int hasWriteExternalStoragePermission = checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (hasWriteExternalStoragePermission != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    REQUEST_CODE_ASK_PERMISSIONS);
            return;
        }
    }

    // Permissions for writing to file
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

    // Inflate menu
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.settings_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle item selection
        switch (item.getItemId()) {
            case R.id.settings:
                startActivity(new Intent(this, SettingsActivity.class));
            default:
                return super.onOptionsItemSelected(item);
        }
    }


}
