package com.bedmas.ash.the_bedmas_game;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;


public class IntroActivity extends ActionBarActivity {

    private int userDifficulty;
    private boolean showAnswers, useTimer;
    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String TAG = "IntroActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_intro);
        setTitle("The BEDMAS Game");

        // get variables from savedInstanceState
        if(savedInstanceState != null)
        {
            userDifficulty = savedInstanceState.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = savedInstanceState.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = savedInstanceState.getBoolean(EXTRA_TIME_TRIALS);
        }

        Intent intent = getIntent();
        if (intent != null)
        {
            Bundle extras = intent.getExtras();
            if (extras != null)
            {
                userDifficulty = extras.getInt(EXTRA_USER_DIFFICULTY, 1);
                showAnswers = extras.getBoolean(EXTRA_SHOW_ANSWERS, true);
                useTimer = extras.getBoolean(EXTRA_TIME_TRIALS, true);
                Log.d(TAG, "from intent useTimer = " + useTimer);
            }
            else
            {
                userDifficulty = 1;
                showAnswers = true;
                useTimer = true;
            }
        }

        ScoreKeeper.get(getApplication());

        Button instructionsButton = (Button) findViewById(R.id.button_instructions);
        instructionsButton.setBackgroundColor(getResources().getColor(R.color.light_teal));
        instructionsButton.setTextColor(getResources().getColor(R.color.white));
        instructionsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent i = new Intent(IntroActivity.this, InstructionsActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
            }
        });

        Button playButton = (Button) findViewById(R.id.button_play);
        playButton.setBackgroundColor(getResources().getColor(R.color.dark_teal));
        playButton.setTextColor(getResources().getColor(R.color.white));
        playButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ScoreKeeper.setScoreMode(false);
                Intent i = new Intent(IntroActivity.this, GameActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
            }
        });

        Button settingsButton = (Button) findViewById(R.id.button_settings);
        settingsButton.setBackgroundColor(getResources().getColor(R.color.med_teal));
        settingsButton.setTextColor(getResources().getColor(R.color.white));
        settingsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent i = new Intent(IntroActivity.this, SettingsActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
            }
        });

        Button playScoredButton = (Button) findViewById(R.id.button_play_scored);
        playScoredButton.setBackgroundColor(getResources().getColor(R.color.dark_dark_teal));
        playScoredButton.setTextColor(getResources().getColor(R.color.white));
        playScoredButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ScoreKeeper.setScoreMode(true);
                Intent i = new Intent(IntroActivity.this, ScoredRoundActivity.class);
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
        getMenuInflater().inflate(R.menu.menu_intro, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case R.id.menu_view_top10:
                finish();
                Intent i = new Intent(this, Top10ScoresActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        savedInstanceState.putInt(EXTRA_USER_DIFFICULTY, userDifficulty);
        savedInstanceState.putBoolean(EXTRA_SHOW_ANSWERS, showAnswers);
        savedInstanceState.putBoolean(EXTRA_TIME_TRIALS, useTimer);
    }
}
