package com.bedmas.ash.the_bedmas_game;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import org.w3c.dom.Text;


public class ScoredRoundActivity extends ActionBarActivity {

    private int userDifficulty, scoredRoundTime;    //scoredRoundTime is in minutes
    private boolean showAnswers, useTimer;
    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String EXTRA_SCORED_ROUND_TIME = "com.bedmas.ash.the_bedmas_game.SCORED_ROUND_TIME";
    private final String TAG = "ScoredRoundActivity";

    private EditText enterTime;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_scored_round);
        setTitle("Play a scored round");

        scoredRoundTime = 0;

        // get variables from savedInstanceState
        if(savedInstanceState != null)
        {
            userDifficulty = savedInstanceState.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = savedInstanceState.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = savedInstanceState.getBoolean(EXTRA_TIME_TRIALS);
            scoredRoundTime = savedInstanceState.getInt(EXTRA_SCORED_ROUND_TIME);
        }

        Bundle extras = getIntent().getExtras();
        if (extras != null)
        {
            userDifficulty = extras.getInt(EXTRA_USER_DIFFICULTY, 1);
            showAnswers = extras.getBoolean(EXTRA_SHOW_ANSWERS, true);
            useTimer = extras.getBoolean(EXTRA_TIME_TRIALS, true);
        }
        else
        {
            userDifficulty = 1;
            showAnswers = true;
            useTimer = true;
        }

        enterTime = (EditText) findViewById(R.id.time_session);
        enterTime.setHint(Integer.toString(scoredRoundTime));
        enterTime.setHintTextColor(getResources().getColor(R.color.grey));
        enterTime.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    scoredRoundTime = Integer.parseInt(enterTime.getText().toString());
                } catch (NumberFormatException e) {
                    makeAToast("You must enter an integer number");
                    Log.d(TAG, "Not an integer");
                }
            }

            @Override
            public void afterTextChanged(Editable s) {
                try {
                    scoredRoundTime = Integer.parseInt(enterTime.getText().toString());
                } catch (NumberFormatException e) {
                    makeAToast("You must enter an integer number");
                    Log.d(TAG, "Not an integer");
                }
            }
        });

        Button playButton = (Button) findViewById(R.id.button_play_scored_activity);
        playButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!enterTime.getText().toString().isEmpty()) scoredRoundTime = Integer.parseInt(enterTime.getText().toString());
                else scoredRoundTime = -1;
                if (enterTime.getText().toString().isEmpty() | scoredRoundTime == -1)
                {
                    makeAToast("Please enter the time of the round");
                }
                else if (scoredRoundTime > 30)
                {
                    makeAToast("The maximum round time is 30 minutes");
                }
                else if (scoredRoundTime <= 0)
                {
                    makeAToast("Round time must be greater than 0");
                }
                else
                {
                    Intent i = new Intent(ScoredRoundActivity.this, GameActivity.class);
                    i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                    i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                    i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                    i.putExtra(EXTRA_SCORED_ROUND_TIME, scoredRoundTime);
                    startActivity(i);
                }
            }
        });


    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_scored_round, menu);
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
        savedInstanceState.putInt(EXTRA_USER_DIFFICULTY, userDifficulty);
        savedInstanceState.putBoolean(EXTRA_SHOW_ANSWERS, showAnswers);
        savedInstanceState.putBoolean(EXTRA_TIME_TRIALS, useTimer);
        savedInstanceState.putInt(EXTRA_SCORED_ROUND_TIME, scoredRoundTime);
    }

    private void makeAToast (String toast) { Toast.makeText(this, toast, Toast.LENGTH_SHORT).show(); }
}
