package com.bedmas.ash.the_bedmas_game;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.Spinner;


public class SettingsActivity extends ActionBarActivity {

    private int userDifficulty;
    private boolean showAnswers, useTimer;

    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String TAG = "SettingsActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);
        setTitle("Settings");

        // get variables from savedInstanceState
        if(savedInstanceState != null)
        {
            userDifficulty = savedInstanceState.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = savedInstanceState.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = savedInstanceState.getBoolean(EXTRA_TIME_TRIALS);
            Log.d(TAG, "savedInstanceState NOT null");
            Log.d(TAG, "showAnswers = " + showAnswers);
            Log.d(TAG, "useTimer = " + useTimer);
        }

        //get variables from intro
        Bundle extras = getIntent().getExtras();
        if (extras != null)
        {
            userDifficulty = extras.getInt(EXTRA_USER_DIFFICULTY, 1);
            showAnswers = extras.getBoolean(EXTRA_SHOW_ANSWERS, true);
            useTimer = extras.getBoolean(EXTRA_TIME_TRIALS, true);
            Log.d(TAG, "extras NOT null");
            Log.d(TAG, "showAnswers = " + showAnswers);
            Log.d(TAG, "useTimer = " + useTimer);
        }
        else
        {
            userDifficulty = 1;
            showAnswers = true;
            useTimer = true;
            Log.d(TAG, "extras IS null");
            Log.d(TAG, "showAnswers = " + showAnswers);
            Log.d(TAG, "useTimer = " + useTimer);
        }

        Spinner difficultySpinner = (Spinner) findViewById(R.id.dropdown_difficulty);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this, R.array.difficulty_levels, android.R.layout.simple_spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        difficultySpinner.setAdapter(adapter);
        difficultySpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                userDifficulty = position + 1;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // do nothing
            }
        });

        String level = "";
        switch(userDifficulty){
            case 1: level = "Easy"; break;
            case 2: level = "Medium"; break;
            case 3: level = "Hard"; break;
            default: level = "Easy"; break;
        }

        difficultySpinner.setSelection(adapter.getPosition(level));

        final CheckBox showAnswersCheckBox = (CheckBox) findViewById(R.id.checkbox_show_answers);
        final CheckBox useTimerCheckBox = (CheckBox) findViewById(R.id.checkbox_time_sessions);

        showAnswersCheckBox.setChecked(showAnswers);
        if (!useTimer) showAnswersCheckBox.setEnabled(false);
        showAnswersCheckBox.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showAnswers = showAnswersCheckBox.isChecked();
            }
        });

        useTimerCheckBox.setChecked(useTimer);
        useTimerCheckBox.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                useTimer = useTimerCheckBox.isChecked();
                if (!useTimer)
                {
                    showAnswersCheckBox.setChecked(true);
                    showAnswers = true;
                    showAnswersCheckBox.setEnabled(false);
                }
                else showAnswersCheckBox.setEnabled(true);
            }
        });

        Button okayButton = (Button) findViewById(R.id.button_settings_okay);
        okayButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent i = new Intent(SettingsActivity.this, IntroActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_settings, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        Log.i(TAG, "onSaveInstanceState");
        savedInstanceState.putInt(EXTRA_USER_DIFFICULTY, userDifficulty);
        savedInstanceState.putBoolean(EXTRA_SHOW_ANSWERS, showAnswers);
        savedInstanceState.putBoolean(EXTRA_TIME_TRIALS, useTimer);
    }
}
