package com.usfreu2016.blunavi;


import com.estimote.sdk.eddystone.Eddystone;

import java.util.Comparator;

public class EddystoneComparator implements Comparator<Eddystone> {

    @Override
    public int compare(Eddystone lhs, Eddystone rhs) {
        return rhs.rssi - lhs.rssi;
    }
}
