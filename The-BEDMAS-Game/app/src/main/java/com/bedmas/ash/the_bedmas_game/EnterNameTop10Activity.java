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
import android.widget.Toast;


public class EnterNameTop10Activity extends ActionBarActivity {

    private int userDifficulty;
    private boolean showAnswers, useTimer;
    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String TAG = "EnterNameTop10Activity";

    private String name;

    private EditText enterName;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_enter_name_top10);
        setTitle("Enter your name");

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

        name = "";

        enterName = (EditText) findViewById(R.id.enter_name);
        enterName.setHint("Name");
        enterName.setHintTextColor(getResources().getColor(R.color.grey));
        enterName.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                name = enterName.getText().toString();
            }

            @Override
            public void afterTextChanged(Editable s) {
                name = enterName.getText().toString();
            }
        });

        Button okayButton = (Button) findViewById(R.id.button_enter_name_okay);
        okayButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (name.isEmpty()) makeAToast("Please enter a name");
                else {
                    // start activity to Top 10 Scores
                    ScoreKeeper.addPlayer(name);
                    Intent intent = new Intent(EnterNameTop10Activity.this, Top10ScoresActivity.class);
                    intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                    intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                    intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                    startActivity(intent);
                }
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_enter_name_top10, menu);
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
    }

    private void makeAToast (String toast) { Toast.makeText(this, toast, Toast.LENGTH_SHORT).show(); }

    @Override
    public void onBackPressed()
    {

    }
}
