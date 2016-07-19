package com.usfreu2016.blunavi;


import android.util.Log;

import com.estimote.sdk.eddystone.Eddystone;

import java.util.Random;

public class Utils {

    private static final String TAG = "Utils";

    private static final double DEGREE = 57.2958;

    public static int computeBearing(double lat1, double lon1, double lat2, double lon2) {

        // convert to rads
        lat1 = lat1 / DEGREE;
        lon1 = lon1 / DEGREE;
        lat2 = lat2/ DEGREE;
        lon2 = lon2 / DEGREE;

        int bearing;

        double deltaLon = lon2 - lon1;
        double x = Math.cos(lat2) * Math.sin(deltaLon);
        double y = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(deltaLon);
        bearing = (int) (Math.atan2(x, y) * DEGREE);

        if (bearing < 0) {
            bearing = 360 + bearing;
        }

        return bearing;

    }

    public static double computeBeaconDistance(Eddystone eddystone) {
        Random rand = new Random();
        double gaussianRandom = rand.nextGaussian() * 4.118278; // std. dev. of beacon rssi values
        int rssi = eddystone.rssi;

        // rssi at 1m away from beacon
        int referenceRssi = -62;
        double exponent = (rssi - referenceRssi - gaussianRandom) / (-10 * 2); // 2 is path loss exponent
        double distance = Math.pow(10, exponent);
        return distance;
    }

}
