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

    public double getXPos() {
        return this.xPos;
    }

    public double getYPos() {
        return this.yPos;
    }

    public double[] getCoords() {
        double[] coords = {xPos, yPos};
        return coords;
    }

    public void setId(String id) {
        this.phyID = id;
    }

    public void setMacAddress(String mac) {
        this.macAddress = mac;
    }

    public void setXPos(double x) {
        this.xPos = x;
    }

    public void setYPos(double y) {
        this.yPos = y;
    }

    public String toString() {
        return "MAC: " + this.macAddress + " x: " + this.xPos + " y: " + yPos;
    }

}
