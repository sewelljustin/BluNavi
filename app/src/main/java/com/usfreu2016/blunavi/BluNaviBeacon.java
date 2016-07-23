package com.usfreu2016.blunavi;


public class BluNaviBeacon {

    private String phyID;
    private String macAddress;
    private double xPos;
    private double yPos;

    public BluNaviBeacon(String id, String macAddress, double x, double y) {
        this.phyID = id;
        this.macAddress = macAddress;
        this.xPos = x;
        this.yPos = y;
    }

    public String getId() {
        return this.phyID;
    }

    public String getMacAddress() {
        return this.macAddress;
    }

    public double[] getCoords() {
        double[] coords = {xPos, yPos};
        return coords;
    }

}
