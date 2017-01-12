package com.bedmas.ash.the_bedmas_game;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.w3c.dom.Text;


public class InstructionsActivity extends ActionBarActivity {

    private String basicInstructions =
            "Using BEDMAS (Brackets, Exponents, Division, Multiplication, Addition & Subtraction), make the three Operand cards " +
                    "at the top equal to the Answer card on the bottom! You must consider the order of operations (ex. exponents have a " +
                    "higher priority than addition) and use brackets where appropriate. The answer equation must be expressed in its most" +
                    "simplified form. Each card is valued at its number (1, 2, 3, etc), while Jack, Queen and King cards are valued at 11, 12 and " +
                    "13, respecitively.";

    private String  levelInstructions =
            "While most trials have a solution, some do not (not possible, or NP). The easy level has no NP problems, while the Medium level has some NP " +
                    "problems, and the Hard level has " +
                    "more NP problems. If you suspect a trial is NP, you can declare so with the NP button. In addition, your time limit decreases " +
                    "as the difficulty increases (Easy = 60 sec, Med = 45 sec, Hard = 30 sec).";

    private String roundingInstructions =
            "Trials that involve divisions will round to the nearest integer. The quotient of each division operation will be rounded before it " +
                    "proceeds to the next operation (ie. the final answer is not necessarily rounded).";

    private String scoringInstructions =
            "For each scored round, the score is proportional to: (time remaining per problem) * level / (time of session). The score is meant to reflect your efficiency. " +
                    "Skipping a round results in a 30 point deduction, while wrong answers result in a 10 point deduction.";


    private int userDifficulty;
    private boolean showAnswers, useTimer;

    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String TAG = "InstructionsActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_instructions);
        setTitle("Instructions");

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

        TextView basic = (TextView) findViewById(R.id.text_basic);
        basic.setText(basicInstructions);

        TextView levels = (TextView) findViewById(R.id.text_levels);
        levels.setText(levelInstructions);

        TextView rounding = (TextView) findViewById(R.id.text_rounding);
        rounding.setText(roundingInstructions);

        TextView scoring = (TextView) findViewById(R.id.text_scoring);
        scoring.setText(scoringInstructions);

        Button okayButton = (Button) findViewById(R.id.button_instructions_okay);
        okayButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent i = new Intent(InstructionsActivity.this ,IntroActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                startActivity(i);
                finish();
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_instructions, menu);
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
