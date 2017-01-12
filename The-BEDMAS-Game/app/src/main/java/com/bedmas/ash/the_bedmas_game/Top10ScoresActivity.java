package com.bedmas.ash.the_bedmas_game;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;

import org.w3c.dom.Text;


public class Top10ScoresActivity extends ActionBarActivity {

    private int userDifficulty;
    private boolean showAnswers, useTimer;
    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String TAG = "Top10ScoresActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_top10_scores);
        setTitle("Top 10 Scores");

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
            }
            else
            {
                userDifficulty = 1;
                showAnswers = true;
                useTimer = true;
            }
        }

        TextView players = (TextView) findViewById(R.id.top10_players);
        players.setText(ScoreKeeper.generatePlayersString());

        TextView scores = (TextView) findViewById(R.id.top10_scores);
        scores.setText(ScoreKeeper.generateScoresString());

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_top10_scores, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case android.R.id.home:
                Intent i = new Intent(this, IntroActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
                //NavUtils.navigateUpFromSameTask(this);
                return true;
            case R.id.menu_item_home:
                Intent iHome = new Intent(this, IntroActivity.class);
                iHome.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                iHome.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                iHome.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(iHome);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }

    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent i = new Intent(Top10ScoresActivity.this, IntroActivity.class);
        i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
        i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
        i.putExtra(EXTRA_TIME_TRIALS, useTimer);
        startActivity(i);
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        savedInstanceState.putInt(EXTRA_USER_DIFFICULTY, userDifficulty);
        savedInstanceState.putBoolean(EXTRA_SHOW_ANSWERS, showAnswers);
        savedInstanceState.putBoolean(EXTRA_TIME_TRIALS, useTimer);
    }
}
