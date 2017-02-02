package com.usfreu2016.blunavi;


import android.app.Dialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.annotation.Nullable;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashSet;


public class SettingsActivity extends AppCompatActivity{

    private String TAG = "SettingsActivity";

    private SharedPreferences settings;

    private ArrayList<BluNaviBeacon> beaconArray;
    private BeaconAdapter beaconArrayAdapter;
    private ListView beaconListView;
    private Button addbeaconButton;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.settings_activity);

        init();
    }

    @Override
    protected void onPause() {
        super.onPause();

        Log.d(TAG, "onPause");

        // persist beacon objects
        SharedPreferences.Editor editor  = settings.edit();

        // create hashset of beacon ids
        HashSet<String> beacon_mac_hashset = new HashSet<>();

        for (BluNaviBeacon beacon : beaconArray) {
            String macAddress = beacon.getMacAddress();
            beacon_mac_hashset.add(macAddress);
            editor.putString(macAddress + "_id", beacon.getId());
            editor.putString(macAddress + "_mac", beacon.getMacAddress());
            editor.putFloat(macAddress + "_x", (float) beacon.getXPos());
            editor.putFloat(macAddress + "_y", (float) beacon.getYPos());
        }

        editor.putStringSet(getString(R.string.beacon_mac_set), beacon_mac_hashset);
        editor.commit();
    }

    private void init() {

        // setup SharedPreferences
        settings = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());

        // initialize beacon ids
        HashSet<String> beacon_mac_hashset = (HashSet<String>) settings.getStringSet(getString(R.string.beacon_mac_set), new HashSet<String>());

        // initialize beaconArray
        beaconArray = new ArrayList<>();

        // populate beacon array with saved beacons
        for (String macAddress: beacon_mac_hashset) {
            String beacon_id = settings.getString(macAddress + "_id", "null");
            String beacon_mac = settings.getString(macAddress + "_mac", "null");
            double beacon_x_loc = settings.getFloat(macAddress + "_x", 0);
            double beacon_y_loc = settings.getFloat(macAddress + "_y", 0);
            final BluNaviBeacon beacon = new BluNaviBeacon(beacon_id, beacon_mac, beacon_x_loc, beacon_y_loc);
            beaconArray.add(beacon);
        }

        // initialize adapter
        beaconArrayAdapter = new BeaconAdapter(this, beaconArray);

        // initialize list view
        beaconListView = (ListView) findViewById(R.id.beacon_list_view);
        beaconListView.setAdapter(beaconArrayAdapter);

        // update total_beacons_textview
        final TextView total_beacons_textview = (TextView) findViewById(R.id.total_beacons_textview);
        total_beacons_textview.setText(String.valueOf(beaconArray.size()));


        // set listview item listener and display dialog
        beaconListView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                final BluNaviBeacon beacon = beaconArrayAdapter.getItem(position);
                EditText beacon_id_edit_text = (EditText) view.findViewById(R.id.beacon_id);
                EditText beacon_mac_edit_text = (EditText) view.findViewById(R.id.beacon_mac);
                EditText beacon_x_loc_edit_text = (EditText) view.findViewById(R.id.beacon_x_location);
                EditText beacon_y_loc_edit_text = (EditText) view.findViewById(R.id.beacon_y_location);

                String beacon_id = beacon.getId();
                String beacon_mac = beacon.getMacAddress();
                String beacon_x_loc = String.valueOf(beacon.getXPos());
                String beacon_y_loc = String.valueOf(beacon.getYPos());

                AlertDialog.Builder builder = new AlertDialog.Builder(SettingsActivity.this);
                builder.setTitle("Edit beacon");
                builder.setView(R.layout.add_beacon_dialog);
                builder.setPositiveButton("Save", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        // Save beacon to shared preferences
                        Dialog d = (Dialog) dialog;

                        EditText beacon_id_edit_text = (EditText) d.findViewById(R.id.beacon_id);
                        EditText beacon_mac_edit_text = (EditText) d.findViewById(R.id.beacon_mac);
                        EditText beacon_x_loc_edit_text = (EditText) d.findViewById(R.id.beacon_x_location);
                        EditText beacon_y_loc_edit_text = (EditText) d.findViewById(R.id.beacon_y_location);

                        String beacon_id = beacon_id_edit_text.getText().toString();
                        String beacon_mac = beacon_mac_edit_text.getText().toString();
                        double beacon_x_loc = Double.parseDouble(beacon_x_loc_edit_text.getText().toString());
                        double beacon_y_loc = Double.parseDouble(beacon_y_loc_edit_text.getText().toString());

                        beacon.setId(beacon_id);
                        beacon.setMacAddress(beacon_mac);
                        beacon.setXPos(beacon_x_loc);
                        beacon.setYPos(beacon_y_loc);

                    }
                });

                builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        // Cancel Dialog
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();

                // Populate dialog with beacon properties
                EditText dialog_beacon_id_edit_text = (EditText) dialog.findViewById(R.id.beacon_id);
                EditText dialog_beacon_mac_edit_text = (EditText) dialog.findViewById(R.id.beacon_mac);
                EditText dialog_beacon_x_loc_edit_text = (EditText) dialog.findViewById(R.id.beacon_x_location);
                EditText dialog_beacon_y_loc_edit_text = (EditText) dialog.findViewById(R.id.beacon_y_location);

                dialog_beacon_id_edit_text.setText(beacon_id);
                dialog_beacon_mac_edit_text.setText(beacon_mac);
                dialog_beacon_x_loc_edit_text.setText(beacon_x_loc);
                dialog_beacon_y_loc_edit_text.setText(beacon_y_loc);

            }
        });

        beaconListView.setOnItemLongClickListener(new AdapterView.OnItemLongClickListener() {
            @Override
            public boolean onItemLongClick(AdapterView<?> parent, View view, final int position, long id) {
                AlertDialog.Builder builder = new AlertDialog.Builder(SettingsActivity.this);
                builder.setTitle("Delete beacon");
                builder.setMessage("Would you like to delete this beacon?");
                builder.setPositiveButton("Delete", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {

                        // remove beacon from array
                        beaconArray.remove(position);
                        beaconArrayAdapter.notifyDataSetChanged();
                        total_beacons_textview.setText(String.valueOf(beaconArray.size()));
                    }
                });
                builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        // cancel dialog
                    }
                });
                AlertDialog dialog = builder.create();
                dialog.show();

                return true;
            }
        });

        addbeaconButton = (Button) findViewById(R.id.add_beacon_button);
        addbeaconButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Create dialog to add beacon

                AlertDialog.Builder builder = new AlertDialog.Builder(SettingsActivity.this);
                builder.setTitle("Add beacon");
                builder.setView(R.layout.add_beacon_dialog);
                builder.setPositiveButton("Save", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            // Save beacon to shared preferences
                            Dialog d = (Dialog) dialog;

                            EditText beacon_id_edit_text = (EditText) d.findViewById(R.id.beacon_id);
                            EditText beacon_mac_edit_text = (EditText) d.findViewById(R.id.beacon_mac);
                            EditText beacon_x_loc_edit_text = (EditText) d.findViewById(R.id.beacon_x_location);
                            EditText beacon_y_loc_edit_text = (EditText) d.findViewById(R.id.beacon_y_location);

                            String beacon_id = beacon_id_edit_text.getText().toString();
                            String beacon_mac = beacon_mac_edit_text.getText().toString();
                            double beacon_x_loc = Double.parseDouble(beacon_x_loc_edit_text.getText().toString());
                            double beacon_y_loc = Double.parseDouble(beacon_y_loc_edit_text.getText().toString());

                            // create new beacon
                            BluNaviBeacon beacon = new BluNaviBeacon(beacon_id, beacon_mac, beacon_x_loc, beacon_y_loc);
                            beaconArrayAdapter.add(beacon);
                            beaconArrayAdapter.notifyDataSetChanged();
                            total_beacons_textview.setText(String.valueOf(beaconArray.size()));
                        }
                    });

                builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            // Cancel Dialog
                        }
                    });

                AlertDialog dialog = builder.create();
                dialog.show();

            }
        });
    }
}
