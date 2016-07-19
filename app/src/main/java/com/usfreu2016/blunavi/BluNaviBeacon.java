package com.usfreu2016.blunavi;


public class BluNaviBeacon {

    private String phyID;
    private String macAddress;
    private int xPos;
    private int yPos;

    public BluNaviBeacon(String id, String macAddress, int x, int y) {
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

    public int[] getCoords() {
        int[] coords = {xPos, yPos};
        return coords;
    }

}
