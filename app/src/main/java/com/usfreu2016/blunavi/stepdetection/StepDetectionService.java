package com.usfreu2016.blunavi.stepdetection;


import android.app.Service;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;

import com.usfreu2016.blunavi.Constants;

public class StepDetectionService extends Service implements SensorEventListener {

    private final String TAG = "StepDetectionService";

    private final Messenger messenger = new Messenger(new StepDetectionService.IncomingHandler());
    private Messenger stepReplyTo;
    private SensorManager sensorManager;
    private StepDetector stepDetector;
    private boolean bound;

    public StepDetectionService() {

    }

    @Override
    public void onCreate() {
        super.onCreate();
        this.bound = false;
        this.sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
    }

    @Override
    public IBinder onBind(Intent intent) {
        this.bound = true;
        return this.messenger.getBinder();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_LINEAR_ACCELERATION:
//                checkData(event.values);

                this.stepDetector.check(event.values);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * Start step detection
     */
    private void startDetection() {
        this.registerSensors();
        this.stepDetector = new StepDetector();
        this.stepDetector.setStepDetectedListener(new StepDetector.StepDetectedListener() {
            @Override
            public void onStepDetected(Step step) {
                try {
                    Message e = Message.obtain(null, Constants.STEP_DETECTED_MSG);
                    Bundle b = new Bundle();
                    b.putParcelable(Constants.STEP_RESULT_KEY, step);
                    e.setData(b);
                    StepDetectionService.this.stepReplyTo.send(e);
                } catch (RemoteException e) {
                    Log.e(TAG, "Error sending step detected message");
                }
            }
        });
    }

    /**
     * Register the sensors used by the service
     */
    private void registerSensors() {
        this.sensorManager.registerListener(
                this, sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION), SensorManager.SENSOR_DELAY_FASTEST);
    }

    /**
     * Unregister sensors used by the service
     */
    private void unregisterSensors() {
        this.sensorManager.unregisterListener(this);
    }

    /**
     * Handles incoming messages from StepDetectionManager
     */
    private class IncomingHandler extends Handler {
        private IncomingHandler() {
        }

        public void handleMessage(Message msg) {
            switch (msg.what) {
                case Constants.REGISTER_MESSENGER_MSG:
                    StepDetectionService.this.stepReplyTo = msg.replyTo;
                    break;
                case Constants.START_STEP_DETECTION_MSG:
                    StepDetectionService.this.startDetection();
                    break;
                case Constants.SET_VERT_ACC_THRESHOLD_MSG:
                    double vertAccThreshold = msg.getData().getDouble(Constants.VERT_THRESHOLD_KEY);
                    StepDetectionService.this.stepDetector.setVertAccThreshold(vertAccThreshold);
                    break;
                case Constants.SET_HORI_ACC_THRESHOLD_MSG:
                    double horiAccThreshold = msg.getData().getDouble(Constants.HORI_THRESHOLD_KEY);
                    StepDetectionService.this.stepDetector.setHoriAccThreshold(horiAccThreshold);
                    break;
                case Constants.SET_STEP_TIME_THRESHOLD_MSG:
                    long stepTimeThreshold = msg.getData().getLong(Constants.STEP_TIME_THRESHOLD_KEY);
                    StepDetectionService.this.stepDetector.setStepTimeThreshold(stepTimeThreshold);
                    break;
                default:
                    break;
            }

        }
    }
}
