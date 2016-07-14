package com.usfreu2016.blunavi.stepdetection;


import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;

import com.usfreu2016.blunavi.Constants;

public class StepDetectionManager {

    private final String TAG = "StepDetectionManager";

    private final Context context;
    private InternalServiceConnection serviceConnection;
    private final Messenger incomingMessenger;
    private Messenger serviceMessenger;
    private StepDetectionManager.ServiceReadyCallback callback;
    private StepDetectionManager.StepListener stepListener;

    /**
     * public Constructor
     * @param context
     */
    public StepDetectionManager(Context context) {
        this.context = context;
        this.serviceConnection = new StepDetectionManager.InternalServiceConnection();
        this.incomingMessenger = new Messenger(new StepDetectionManager.IncomingHandler());
    }

    /**
     * Connects StepDetectionManager to StepDetectionService
     *
     * @param callback Service ready callback
     */
    public void connect(StepDetectionManager.ServiceReadyCallback callback) {
        this.callback = callback;
        if(this.isConnectedToService()) {
            callback.onServiceReady();
        }

        boolean bound = this.context.bindService(new Intent(this.context, StepDetectionService.class), this.serviceConnection, Context.BIND_AUTO_CREATE);
    }

    /**
     * Disconnects StepDetectionManager from StepDetectionService
     */
    public void disconnect() {
        if(this.isConnectedToService()) {

            this.context.unbindService(this.serviceConnection);
            this.callback = null;
            this.stepListener = null;
            this.serviceMessenger = null;

        }
    }

    /**
     * Set the vertical acceleration value for step detection
     * @param threshold The vertical acceleration threshold value
     */
    public void setVertAccThreshold(float threshold) {
        if (this.isConnectedToService()) {
            Message setVertThresholdMsg =
                    Message.obtain(null, Constants.SET_VERT_ACC_THRESHOLD_MSG);
            Bundle b = new Bundle();
            b.putFloat(Constants.VERT_THRESHOLD_KEY, threshold);
            setVertThresholdMsg.setData(b);
            try {
                this.serviceMessenger.send(setVertThresholdMsg);
            } catch (RemoteException e) {
                Log.e(TAG, "Error sending start step detection message to service.");
            }
        }
    }

    /**
     * Check if service is connected
     * @return True if connected to service
     */
    private boolean isConnectedToService() {
        return this.serviceMessenger != null;
    }

    /**
     * Sets a step listener
     * @param stepListener Listener for step event
     */
    public void setStepListener(StepDetectionManager.StepListener stepListener) {
        this.stepListener = stepListener;
    }

    /**
     * Starts step detection in service
     */
    public void startStepDetection() {
        if (this.isConnectedToService()) {
            Message startMsg = Message.obtain(null, Constants.START_STEP_DETECTION_MSG);

            try {
                this.serviceMessenger.send(startMsg);
            } catch (RemoteException e) {
                Log.e(TAG, "Error sending start step detection message to service.");
            }
        }
    }

    /**
     * Registers a step listener in the StepDetectionService
     */
    private void registerMessengerInService() {
        Message registerMsg = Message.obtain(null, Constants.REGISTER_MESSENGER_MSG);
        registerMsg.replyTo = this.incomingMessenger;

        try {
            this.serviceMessenger.send(registerMsg);
        } catch (RemoteException e) {

        }

    }

    /**
     * Handles incoming messages from StepDetectionService
     */
    private class IncomingHandler extends Handler {
        private IncomingHandler() {
        }

        public void handleMessage(Message msg) {
            switch(msg.what) {
                case Constants.STEP_DETECTED_MSG:
                    if(StepDetectionManager.this.stepListener != null) {
                        Step step = msg.getData().getParcelable(Constants.STEP_RESULT_KEY);
                        StepDetectionManager.this.stepListener.onStep(step);
                    }
                    break;
                default:
                    break;
            }

        }
    }

    /**
     * Handles connection to StepDetectionService
     */
    private class InternalServiceConnection implements ServiceConnection {
        private InternalServiceConnection() {
        }

        public void onServiceConnected(ComponentName name, IBinder service) {
            StepDetectionManager.this.serviceMessenger = new Messenger(service);

            if(StepDetectionManager.this.stepListener!= null) {
                StepDetectionManager.this.registerMessengerInService();
            }

            if(StepDetectionManager.this.callback != null) {
                StepDetectionManager.this.callback.onServiceReady();
                StepDetectionManager.this.callback = null;
            }

        }

        public void onServiceDisconnected(ComponentName name) {
            StepDetectionManager.this.serviceMessenger = null;
        }
    }

    /**
     * Interface for StepListener
     */
    public interface StepListener {
        void onStep(Step step);
    }

    /**
     * Callback triggered when StepDetectionService is ready
     */
    public interface ServiceReadyCallback {
        void onServiceReady();
    }
}
