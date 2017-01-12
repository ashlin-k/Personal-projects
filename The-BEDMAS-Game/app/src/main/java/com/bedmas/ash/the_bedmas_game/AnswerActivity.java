package com.bedmas.ash.the_bedmas_game;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.res.Configuration;
import android.graphics.Point;
import android.support.v4.app.NavUtils;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;


public class AnswerActivity extends ActionBarActivity {

    private final String EXTRA_ANSWER_STRING = "com.bedmas.ash.the_bedmas_game.ANSWER_STRING";
    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String EXTRA_SUITS = "com.bedmas.ash.the_bedmas_game.SUITS";
    private final String EXTRA_VALUES = "com.bedmas.ash.the_bedmas_game.VALUES";
    private final String TAG = "AnswerActivity";
    private String answerDialogText;
    private int userDifficulty;
    private boolean showAnswers, useTimer;
    private Operand op1, op2, op3;
    private Quotient qu;

    //private String testString = "Hi!\n\n\n\n\n\n\nHi!\n\n\n\n\n\n\n\n\nHi\n\n\n\n\n\n\nHi\n\n\n\n\n\n\n\n\n\n\nmany lines\n\n\n\n\n\n\n\n\n\nmany many lines";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_answer);
        setTitle("Answers");

        ImageView o1, o2, o3, q;

        // get variables from savedInstanceState
        if(savedInstanceState != null)
        {
            userDifficulty = savedInstanceState.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = savedInstanceState.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = savedInstanceState.getBoolean(EXTRA_TIME_TRIALS);
        }

        Bundle extras = getIntent().getExtras();
        if (extras != null)
        {
            answerDialogText = extras.getString(EXTRA_ANSWER_STRING);
            userDifficulty = extras.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = extras.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = extras.getBoolean(EXTRA_TIME_TRIALS);
            char[] suits = extras.getCharArray(EXTRA_SUITS);
            int[] vals = extras.getIntArray(EXTRA_VALUES);

            Log.d(TAG, "showAnswers = " + showAnswers);
            Log.d(TAG, "useTimer = " + useTimer);

            if (suits != null & vals != null)
            {
                op1 = new Operand(new Card(suits[0], vals[0]));
                op2 = new Operand(new Card(suits[1], vals[1]));
                op3 = new Operand(new Card(suits[2], vals[2]));
                qu = new Quotient(new Card(suits[3], vals[3]));
            }
        }
        else
        {
            answerDialogText = "";
            userDifficulty = -1;
            showAnswers = true;
            useTimer = true;
        }

        if (op1 != null)
        {
            o1 = (ImageView) findViewById(R.id.ans_op1);
            o1.setImageResource(getCardResId(op1.getSuit(), op1.getValue()));
            o2 = (ImageView) findViewById(R.id.ans_op2);
            o2.setImageResource(getCardResId(op2.getSuit(), op2.getValue()));
            o3 = (ImageView) findViewById(R.id.ans_op3);
            o3.setImageResource(getCardResId(op3.getSuit(), op3.getValue()));
            q = (ImageView) findViewById(R.id.ans_qu);
            q.setImageResource(getCardResId(qu.getSuit(), qu.getValue()));

            int screenHeight, screenWidth;
            WindowManager windowManager = (WindowManager) getApplicationContext().getSystemService(Context.WINDOW_SERVICE);
            Display display = windowManager.getDefaultDisplay();
            if (android.os.Build.VERSION.SDK_INT>=13) {
                Point size = new Point();
                display.getSize(size);
                screenHeight = size.x;
                //screenWidth = size.y;
            }
            else {
                //screenWidth = display.getWidth();
                screenHeight = display.getHeight();
            }

            if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
                o1.getLayoutParams().height = (int) screenHeight / 8;
                o2.getLayoutParams().height = (int) screenHeight / 8;
                o3.getLayoutParams().height = (int) screenHeight / 8;
                q.getLayoutParams().height = (int) screenHeight / 8;
            }
            else
            {
                o1.getLayoutParams().height = (int) screenHeight / 4;
                o2.getLayoutParams().height = (int) screenHeight / 4;
                o3.getLayoutParams().height = (int) screenHeight / 4;
                q.getLayoutParams().height = (int) screenHeight / 4;
            }

            o1.requestLayout();
            o2.requestLayout();
            o3.requestLayout();
            q.requestLayout();

        }

        TextView ans = (TextView) findViewById(R.id.answer);
        ans.setText(answerDialogText);

        Button playAgain = (Button) findViewById(R.id.play_again);
        playAgain.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent i = new Intent(AnswerActivity.this, GameActivity.class);
                i.putExtra(EXTRA_SHOW_ANSWERS, true);
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
        getMenuInflater().inflate(R.menu.menu_answer, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case android.R.id.home:
                finish();
                //NavUtils.navigateUpFromSameTask(this);
                Intent iHome = new Intent(this, IntroActivity.class);
                iHome.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                iHome.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                iHome.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(iHome);
                return true;
            case R.id.menu_item_home_aa:
                finish();
                Intent i = new Intent(this, IntroActivity.class);
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
    public void onBackPressed()
    {
        super.onBackPressed();
        finish();
        Intent i = new Intent(AnswerActivity.this, IntroActivity.class);
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

    public int getCardResId(char suit, int value)
    {
        switch(suit)
        {
            case 'h':
            {
                if (value == 1) return R.drawable.h1;
                else if (value == 2) return R.drawable.h2;
                else if (value == 3) return R.drawable.h3;
                else if (value == 4) return R.drawable.h4;
                else if (value == 5) return R.drawable.h5;
                else if (value == 6) return R.drawable.h6;
                else if (value == 7) return R.drawable.h7;
                else if (value == 8) return R.drawable.h8;
                else if (value == 9) return R.drawable.h9;
                else if (value == 10) return R.drawable.h10;
                else if (value == 11) return R.drawable.h11;
                else if (value == 12) return R.drawable.h12;
                else if (value == 13) return R.drawable.h13;
                else return -1;
            }
            case 'd':
            {
                if (value == 1) return R.drawable.d1;
                else if (value == 2) return R.drawable.d2;
                else if (value == 3) return R.drawable.d3;
                else if (value == 4) return R.drawable.d4;
                else if (value == 5) return R.drawable.d5;
                else if (value == 6) return R.drawable.d6;
                else if (value == 7) return R.drawable.d7;
                else if (value == 8) return R.drawable.d8;
                else if (value == 9) return R.drawable.d9;
                else if (value == 10) return R.drawable.d10;
                else if (value == 11) return R.drawable.d11;
                else if (value == 12) return R.drawable.d12;
                else if (value == 13) return R.drawable.d13;
                else return -1;
            }
            case 's':
            {
                if (value == 1) return R.drawable.s1;
                else if (value == 2) return R.drawable.s2;
                else if (value == 3) return R.drawable.s3;
                else if (value == 4) return R.drawable.s4;
                else if (value == 5) return R.drawable.s5;
                else if (value == 6) return R.drawable.s6;
                else if (value == 7) return R.drawable.s7;
                else if (value == 8) return R.drawable.s8;
                else if (value == 9) return R.drawable.s9;
                else if (value == 10) return R.drawable.s10;
                else if (value == 11) return R.drawable.s11;
                else if (value == 12) return R.drawable.s12;
                else if (value == 13) return R.drawable.s13;
                else return -1;
            }
            case 'c':
            {
                if (value == 1) return R.drawable.c1;
                else if (value == 2) return R.drawable.c2;
                else if (value == 3) return R.drawable.c3;
                else if (value == 4) return R.drawable.c4;
                else if (value == 5) return R.drawable.c5;
                else if (value == 6) return R.drawable.c6;
                else if (value == 7) return R.drawable.c7;
                else if (value == 8) return R.drawable.c8;
                else if (value == 9) return R.drawable.c9;
                else if (value == 10) return R.drawable.c10;
                else if (value == 11) return R.drawable.c11;
                else if (value == 12) return R.drawable.c12;
                else if (value == 13) return R.drawable.c13;
                else return -1;
            }
        }

        return -1;
    }
}
