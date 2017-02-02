package com.usfreu2016.blunavi;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import java.util.ArrayList;

public class BeaconAdapter extends ArrayAdapter<BluNaviBeacon> {
    public BeaconAdapter(Context context, ArrayList<BluNaviBeacon> users) {
        super(context, 0, users);
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {

        BluNaviBeacon beacon = getItem(position);

        if (convertView == null) {
            convertView = LayoutInflater.from(getContext()).inflate(R.layout.beacon_item_layout, parent, false);
        }

        TextView beacon_id_text_view = (TextView) convertView.findViewById(R.id.beacon_id_text_view);
        TextView beacon_mac_text_view = (TextView) convertView.findViewById(R.id.beacon_mac_text_view);
        TextView beacon_x_loc_text_view = (TextView) convertView.findViewById(R.id.beacon_x_location_text_view);
        TextView beacon_y_loc_text_view = (TextView) convertView.findViewById(R.id.beacon_y_location_text_view);

        beacon_id_text_view.setText(beacon.getId());
        beacon_mac_text_view.setText(beacon.getMacAddress());
        beacon_x_loc_text_view.setText(Double.toString(beacon.getCoords()[0]));
        beacon_y_loc_text_view.setText(Double.toString(beacon.getCoords()[1]));


        return convertView;
    }
}
