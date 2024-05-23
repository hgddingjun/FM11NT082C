package com.dingjun.nfcwritedemo;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MainActivity extends AppCompatActivity {
    private final String TAG = "NFCWriteDemo";
    //private EditText editEncrypt;
    private EditText editSSID;
    private EditText editPasswd;
    private Button   btnWriteEeprom;
    private Spinner spinnerEncrypt;

    private int encryptType = 0;
    private String selectItem="";

    private String encryptPath = "/sys/devices/platform/11cb0000.i2c3/i2c-3/3-0057/encrypt";
    private String ssidPath = "/sys/devices/platform/11cb0000.i2c3/i2c-3/3-0057/ssid";
    private String passwordPath = "/sys/devices/platform/11cb0000.i2c3/i2c-3/3-0057/password";
    private String writeEEPROM = "/sys/devices/platform/11cb0000.i2c3/i2c-3/3-0057/dumpE2P";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);
        Log.d(TAG, "onCreate: ");

        spinnerEncrypt = findViewById(R.id.encryptSpinner);

        //editEncrypt = findViewById(R.id.editEncryptText);
        editSSID = findViewById(R.id.editSsidText);
        editPasswd = findViewById(R.id.editPassword);
        btnWriteEeprom = findViewById(R.id.buttonOkay);

        String[] items = {"NONE", "WEP", "WPA/WPA2"};

        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, items);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerEncrypt.setAdapter(adapter);
        spinnerEncrypt.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                selectItem = items[position];
                if( 0 == position ) {
                    encryptType = 1;
                } else if( 1 ==position ) {
                    encryptType = 2;
                } else if( 2 ==position ) {
                    encryptType = 8;
                } else {
                    encryptType = 1;
                }

                Toast.makeText(MainActivity.this, "Select:" + selectItem + "<->Position:" + String.valueOf(position), Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        btnWriteEeprom.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                String encrytoText = selectItem;//editEncrypt.getText().toString();
                String ssidText = editSSID.getText().toString();
                String passwdText = editPasswd.getText().toString();
                Toast.makeText(MainActivity.this, "加密方式:" + encrytoText + " | WIFI热点:" + ssidText+" | 密码:"+passwdText, Toast.LENGTH_LONG).show();
                Log.d(TAG, "onClick: " + "加密方式:"+encrytoText + " | WIFI热点:"+ssidText + " | 密码:" + passwdText);

                try {
                    //writeStr(encrytoText,encryptPath);
                    write(encryptType,encryptPath);
                    Thread.sleep(10);
                    writeStr(ssidText,ssidPath);
                    Thread.sleep(10);
                    writeStr(passwdText,passwordPath);
                    Thread.sleep(10);

                    write(1,writeEEPROM);
                    Thread.sleep(50);
                } catch (Exception e) {
                    Log.e(TAG, "onClick->write path error:" + e.toString());
                    e.printStackTrace();
                }

            }
        });
//        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
//            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
//            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
//            return insets;
//        });
    }

    private void writeStr(String strVal, String strPath) {
        File blCtrlWake = new File(strPath);
        FileWriter fr;

        try {
            fr = new FileWriter(blCtrlWake);
            fr.write(strVal);
            fr.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    private void write(int intValue,String strPath) {
        File blCtrlWake = new File(strPath);
        FileWriter fr;

        try {
            fr = new FileWriter(blCtrlWake);
            fr.write(String.valueOf(intValue));
            fr.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}