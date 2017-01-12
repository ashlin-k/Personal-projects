package com.bedmas.ash.the_bedmas_game;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.res.Configuration;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.media.Image;
import android.os.CountDownTimer;
import android.support.v4.app.NavUtils;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import org.w3c.dom.Text;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;


public class GameActivity extends ActionBarActivity {

    /*
    properties transferred from settings:
    -int userDifficulty
    -boolean showAnswers
    */

    private Random rand;
    private int userDifficulty, scoredRoundTime;
    private Operand op1, op2, op3;
    private Quotient qu;
    private boolean enableEditText, showAnswers, useTimer;
    private String answerString, answerDialogText;
    private TextView answer, timerText, scoreText;
    private Trial trial;
    private ImageView operand1, operand2, operand3;
    private ImageView quotient;
    private CountDownTimer timer;
    private long time;
    private Context context;

    private final String EXTRA_USER_DIFFICULTY = "com.bedmas.ash.the_bedmas_game.USER_DIFFICULTY";
    private final String EXTRA_SHOW_ANSWERS = "com.bedmas.ash.the_bedmas_game.SHOW_ANSWERS";
    private final String EXTRA_ANSWER_STRING = "com.bedmas.ash.the_bedmas_game.ANSWER_STRING";
    private final String EXTRA_TIME_TRIALS = "com.bedmas.ash.the_bedmas_game.TIME_TRIALS";
    private final String EXTRA_TRIAL_PARCEL = "com.bedmas.ash.the_bedmas_game.TRIAL_PARCEL";
    private final String EXTRA_TIME = "com.bedmas.ash.the_bedmas_game.TIME";
    private final String EXTRA_SUITS = "com.bedmas.ash.the_bedmas_game.SUITS";
    private final String EXTRA_VALUES = "com.bedmas.ash.the_bedmas_game.VALUES";
    private final String EXTRA_SCORED_ROUND_TIME = "com.bedmas.ash.the_bedmas_game.SCORED_ROUND_TIME";
    private final String TAG = "GameActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_game);
        setTitle("Let's play!");


        rand = new Random();
        enableEditText = true;
        answerString = "";

        //set up score
        if (ScoreKeeper.getScoreMode())
        {
            scoreText = new TextView(this);
            LinearLayout scorelinlay = (LinearLayout) findViewById(R.id.linlay_timer_score);
            scoreText.setText(Integer.toString(ScoreKeeper.getCurrentScore()));
            scoreText.setTextSize(18);
            scoreText.setTypeface(Typeface.SANS_SERIF, Typeface.BOLD);
            scoreText.setTextColor(getResources().getColor(R.color.dark_red));
            scoreText.setGravity(Gravity.CENTER);
            scoreText.setPadding(200, 0, 0, 0);
            scorelinlay.addView(scoreText);
        }

        //get variables from intro
        Bundle extras = getIntent().getExtras();
        if (extras != null)
        {
            userDifficulty = extras.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = extras.getBoolean(EXTRA_SHOW_ANSWERS);
            useTimer = extras.getBoolean(EXTRA_TIME_TRIALS);
            if (ScoreKeeper.getScoreMode())
            {
                scoredRoundTime = extras.getInt(EXTRA_SCORED_ROUND_TIME, 3);
                ScoreKeeper.resetRound(scoredRoundTime);        //the score is resetting on rotation
            }
        }
        else
        {
            userDifficulty = -1;
            showAnswers = true;
            useTimer = true;
            if (ScoreKeeper.getScoreMode())
            {
                scoredRoundTime = 3;
                ScoreKeeper.resetRound(scoredRoundTime);
            }
        }

        // get variables from savedInstanceState
        if(savedInstanceState != null)
        {
            userDifficulty = savedInstanceState.getInt(EXTRA_USER_DIFFICULTY);
            showAnswers = savedInstanceState.getBoolean(EXTRA_SHOW_ANSWERS);
            answerString = savedInstanceState.getString(EXTRA_ANSWER_STRING);
            useTimer = savedInstanceState.getBoolean(EXTRA_TIME_TRIALS);
            time = savedInstanceState.getLong(EXTRA_TIME);
            if (ScoreKeeper.getScoreMode())
            {
                scoredRoundTime = savedInstanceState.getInt(EXTRA_SCORED_ROUND_TIME);
                if (scoreText != null) scoreText.setText(Integer.toString(ScoreKeeper.getCurrentScore()));
            }
            Log.d(TAG, "getting time after rotation: " + time);
            MyParcelable parce = savedInstanceState.getParcelable(EXTRA_TRIAL_PARCEL);

            if (parce != null)
            {
                op1 = new Operand(new Card(parce.getSuits()[0], parce.getValues()[0]));
                op2 = new Operand(new Card(parce.getSuits()[1], parce.getValues()[1]));
                op3 = new Operand(new Card(parce.getSuits()[2], parce.getValues()[2]));
                qu = new Quotient(new Card(parce.getSuits()[3], parce.getValues()[3]));
                trial = new Trial(op1, op2, op3, qu);
            }
            else trial = randomTrialGenerator();

        }
        else trial = randomTrialGenerator();

        // set background color based on level
        if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT){
            RelativeLayout rellay = (RelativeLayout) findViewById(R.id.rellay_game);
            switch (userDifficulty) {
                case 1: rellay.setBackgroundColor(getResources().getColor(R.color.orange)); break;
                case 2: rellay.setBackgroundColor(getResources().getColor(R.color.blue)); break;
                case 3: rellay.setBackgroundColor(getResources().getColor(R.color.red)); break;
                default: rellay.setBackgroundColor(getResources().getColor(R.color.purple)); break;
            }
        }
        else {
            LinearLayout linlay = (LinearLayout) findViewById(R.id.linlay_game);
            switch (userDifficulty) {
                case 1: linlay.setBackgroundColor(getResources().getColor(R.color.orange)); break;
                case 2: linlay.setBackgroundColor(getResources().getColor(R.color.blue)); break;
                case 3: linlay.setBackgroundColor(getResources().getColor(R.color.red)); break;
                default: linlay.setBackgroundColor(getResources().getColor(R.color.purple)); break;
            }
        }

        if (trial != null)
        {
            operand1 = (ImageView) findViewById(R.id.op1);
            operand1.setImageResource(getCardResId(op1.getSuit(), op1.getValue()));
            operand1.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString(Integer.toString(getOperand(1).getValue()));
                    updateAnswer();
                }
            });

            operand2 = (ImageView) findViewById(R.id.op2);
            operand2.setImageResource(getCardResId(op2.getSuit(), op2.getValue()));
            operand2.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString(Integer.toString(getOperand(2).getValue()));
                    updateAnswer();
                }
            });

            operand3 = (ImageView) findViewById(R.id.op3);
            operand3.setImageResource(getCardResId(op3.getSuit(), op3.getValue()));
            operand3.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString(Integer.toString(getOperand(3).getValue()));
                    updateAnswer();
                }
            });

            quotient = (ImageView) findViewById(R.id.quotient);
            quotient.setImageResource(getCardResId(qu.getSuit(), qu.getValue()));

            answer = (TextView) findViewById(R.id.edittext_answer);
            if (answerString != null) answer.setText(answerString);
            else answer.setText("");

            Button notPossible = (Button) findViewById(R.id.button_not_possible);
            notPossible.setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    if(event.getAction() == MotionEvent.ACTION_DOWN)
                    {
                        if (getEET()) answer.setFocusable(false);
                        else answer.setFocusable(true);
                        toggleEET();
                    }
                    return false;
                }
            });

            Button leftBracketButton = (Button) findViewById(R.id.button_left_bracket);
            Button rightBracketButton = (Button) findViewById(R.id.button_right_bracket);
            Button exponentButton = (Button) findViewById(R.id.button_exponent);
            Button divideButton = (Button) findViewById(R.id.button_divide);
            Button multiplyButton = (Button) findViewById(R.id.button_multiply);
            Button addButton = (Button) findViewById(R.id.button_add);
            Button subtractButton = (Button) findViewById(R.id.button_subtract);
            Button clearButton = (Button) findViewById(R.id.button_clear);
            Button doneButton = (Button) findViewById(R.id.button_done);
            Button npButton = (Button) findViewById(R.id.button_not_possible);

            //dynamically create show answers button if there is no timer
            //if (!useTimer) {
            if (!useTimer & !ScoreKeeper.getScoreMode()) {
                Button showAnswersButton = new Button(this);
                showAnswersButton.setBackgroundColor(getResources().getColor(R.color.shale));
                showAnswersButton.setTextColor(getResources().getColor(R.color.white));
                showAnswersButton.setText("SHOW\nANS");
                final float scale = getResources().getDisplayMetrics().density;
                int padding_20dp = (int) (20 * scale + 0.5f);
                int padding_10dp = (int) (10 * scale + 0.5f);
                showAnswersButton.setPadding(padding_10dp, padding_10dp, padding_10dp, padding_10dp);
                showAnswersButton.setTextSize(10);
                LinearLayout ll = (LinearLayout) findViewById(R.id.linlay_edittext);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
                lp.weight = 1;
                lp.gravity = Gravity.CENTER_VERTICAL;
                lp.rightMargin = 10;
                ll.addView(showAnswersButton, lp);
                showAnswersButton.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        Intent intent = new Intent(GameActivity.this, AnswerActivity.class);
                        intent.putExtra(EXTRA_ANSWER_STRING, answerDialogText);
                        intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                        intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                        intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                        intent.putExtra(EXTRA_SUITS, new char[]{op1.getSuit(), op2.getSuit(), op3.getSuit(), qu.getSuit()});
                        intent.putExtra(EXTRA_VALUES, new int[]{op1.getValue(), op2.getValue(), op3.getValue(), qu.getValue()});
                        startActivity(intent);
                    }
                });
            }

            //dynamically create skip trial button if in score mode
            if (ScoreKeeper.getScoreMode()) {
                Button skipTrialButton = new Button(this);
                skipTrialButton.setBackgroundColor(getResources().getColor(R.color.shale));
                skipTrialButton.setTextColor(getResources().getColor(R.color.white));
                skipTrialButton.setText("SKIP");
                final float scale = getResources().getDisplayMetrics().density;
                int padding_20dp = (int) (20 * scale + 0.5f);
                int padding_10dp = (int) (10 * scale + 0.5f);
                skipTrialButton.setPadding(padding_10dp, padding_10dp, padding_10dp, padding_10dp);
                skipTrialButton.setTextSize(10);
                LinearLayout ll = (LinearLayout) findViewById(R.id.linlay_edittext);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
                lp.weight = 1;
                lp.gravity = Gravity.CENTER_VERTICAL;
                lp.rightMargin = 10;
                ll.addView(skipTrialButton, lp);
                skipTrialButton.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        //deduct points
                        ScoreKeeper.deductPointsForSkip(true);
                        scoreText.setText(Integer.toString(ScoreKeeper.getCurrentScore()));
                        //new trial
                        trial = randomTrialGenerator();
                        updateCards();
                    }
                });
            }

            //configure rest of buttons
            npButton.setBackgroundColor(getResources().getColor(R.color.shale));
            npButton.setTextColor(getResources().getColor(R.color.white));
            npButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    setAnswerString("NP");
                    updateAnswer();
                }
            });

            leftBracketButton.setBackgroundColor(getResources().getColor(R.color.shale));
            leftBracketButton.setTextColor(getResources().getColor(R.color.white));
            leftBracketButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("(");
                    updateAnswer();
                }
            });

            rightBracketButton.setBackgroundColor(getResources().getColor(R.color.shale));
            rightBracketButton.setTextColor(getResources().getColor(R.color.white));
            rightBracketButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString(")");
                    updateAnswer();
                }
            });

            exponentButton.setBackgroundColor(getResources().getColor(R.color.shale));
            exponentButton.setTextColor(getResources().getColor(R.color.white));
            exponentButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("^");
                    updateAnswer();
                }
            });

            divideButton.setBackgroundColor(getResources().getColor(R.color.shale));
            divideButton.setTextColor(getResources().getColor(R.color.white));
            divideButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("/");
                    updateAnswer();
                }
            });

            multiplyButton.setBackgroundColor(getResources().getColor(R.color.shale));
            multiplyButton.setTextColor(getResources().getColor(R.color.white));
            multiplyButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("*");
                    updateAnswer();
                }
            });

            addButton.setBackgroundColor(getResources().getColor(R.color.shale));
            addButton.setTextColor(getResources().getColor(R.color.white));
            addButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("+");
                    updateAnswer();
                }
            });

            subtractButton.setBackgroundColor(getResources().getColor(R.color.shale));
            subtractButton.setTextColor(getResources().getColor(R.color.white));
            subtractButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    addToAnswerString("-");
                    updateAnswer();
                }
            });

            clearButton.setBackgroundColor(getResources().getColor(R.color.shale));
            clearButton.setTextColor(getResources().getColor(R.color.white));
            clearButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    setAnswerString("");
                    updateAnswer();
                }
            });

            doneButton.setBackgroundColor(getResources().getColor(R.color.shale));
            doneButton.setTextColor(getResources().getColor(R.color.white));
            doneButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (checkAnswer()) {

                        //create a new timer
                        /*if (useTimer & !ScoreKeeper.getScoreMode()) {
                        //if (useTimer) {

                            timer.cancel();

                            //re do the interval for the next round
                            long interval;
                            switch (userDifficulty) {
                                case 1:
                                    interval = 61000;
                                    break;
                                case 2:
                                    interval = 46000;
                                    break;
                                case 3:
                                    interval = 31000;
                                    break;
                                default:
                                    interval = 61000;
                                    break;
                            }

                            timer = new CountDownTimer(interval, 1000) {
                                @Override
                                public void onTick(long millisUntilFinished) {
                                    time = millisUntilFinished;
                                    timerText.setText(Integer.toString((int) (millisUntilFinished / 1000) - 1));

                                }

                                @Override
                                public void onFinish() {
                                    timerText.setText(Integer.toString(0));
                                    makeAToast("Time's up!");
                                    if (showAnswers & !ScoreKeeper.getScoreMode()) {
                                    //if (showAnswers) {
                                        Intent intent = new Intent(GameActivity.this, AnswerActivity.class);
                                        intent.putExtra(EXTRA_ANSWER_STRING, answerDialogText);
                                        intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                                        intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                                        intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                                        intent.putExtra(EXTRA_SUITS, new char[]{op1.getSuit(), op2.getSuit(), op3.getSuit(), qu.getSuit()});
                                        intent.putExtra(EXTRA_VALUES, new int[]{op1.getValue(), op2.getValue(), op3.getValue(), qu.getValue()});
                                        startActivity(intent);
                                    }
                                    else
                                    {
                                        timerText.setText(Integer.toString(0));
                                        trial = randomTrialGenerator();
                                        updateCards();
                                        timer.start();
                                    }
                                }
                            };
                            timer.start();
                        }*/

                        if (ScoreKeeper.getScoreMode() & scoreText != null)
                        {
                            //update score
                            ScoreKeeper.calculateCurrentScore(time, userDifficulty);
                            scoreText.setText(Integer.toString(ScoreKeeper.getCurrentScore()));
                        }

                        trial = randomTrialGenerator();
                        updateCards();
                    }

                    //wrong answer
                    else
                    {
                        if (ScoreKeeper.getScoreMode())
                        {
                            //deduct points
                            ScoreKeeper.deductPointsForSkip(false);
                            scoreText.setText(Integer.toString(ScoreKeeper.getCurrentScore()));
                        }
                    }
                }
            });

        }

        //get correct answers
        answerDialogText = "Accepted answers:\n";
        if (!trial.getAnswerStrings().isEmpty())
            for (int i=0; i<trial.getAnswerStrings().size(); i++) answerDialogText = answerDialogText + trial.getAnswerStrings().get(i) + "\n";
        else answerDialogText = answerDialogText + "No solution exists.";

        //set up timer
        if (useTimer | ScoreKeeper.getScoreMode()) {
        //if (useTimer) {
            timerText = (TextView) findViewById(R.id.timer);
            long interval;
            context = this;

            if (savedInstanceState != null) {
                interval = time;
                Log.d(TAG, "setting interval ssi NOT null: " + interval);
            }
            else if (ScoreKeeper.getScoreMode())
            {
                interval = scoredRoundTime * 60 * 1000 + 200;
            }
            else
            {
                switch (userDifficulty) {
                    case 1:
                        interval = 61000;
                        break;
                    case 2:
                        interval = 46000;
                        break;
                    case 3:
                        interval = 31000;
                        break;
                    default:
                        interval = 61000;
                        break;
                }
                Log.d(TAG, "setting interval ssi IS null: " + interval);
            }

            timer = new CountDownTimer(interval, 1000) {
                @Override
                public void onTick(long millisUntilFinished) {
                    time = millisUntilFinished;
                    timerText.setText(Integer.toString((int) (millisUntilFinished / 1000) - 1));
                }

                @Override
                public void onFinish() {
                    timerText.setText(Integer.toString(0));
                    makeAToast("Time's up!");
                    if (showAnswers & !ScoreKeeper.getScoreMode())
                    //if (showAnswers)
                    {
                        Intent intent = new Intent(GameActivity.this, AnswerActivity.class);
                        intent.putExtra(EXTRA_ANSWER_STRING, answerDialogText);
                        intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                        intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                        intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                        intent.putExtra(EXTRA_SUITS, new char[]{op1.getSuit(), op2.getSuit(), op3.getSuit(), qu.getSuit()});
                        intent.putExtra(EXTRA_VALUES, new int[]{op1.getValue(), op2.getValue(), op3.getValue(), qu.getValue()});
                        startActivity(intent);
                    } else {
                        if (!ScoreKeeper.getScoreMode())
                        {
                            timerText.setText(Integer.toString(0));
                            trial = randomTrialGenerator();
                            updateCards();
                            timer.cancel();
                            timer.start();
                        }
                        else
                        {
                            //add score to top 10
                            ScoreKeeper.endRound();
                            boolean isInTop10 = ScoreKeeper.addToTop10();
                            if (isInTop10)
                            {
                                Intent intent = new Intent(GameActivity.this, EnterNameTop10Activity.class);
                                intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                                intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                                intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                                startActivity(intent);
                            }
                            else
                            {
                                Intent intent = new Intent(GameActivity.this, Top10ScoresActivity.class);
                                intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                                intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                                intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                                startActivity(intent);
                            }
                        }
                    }
                }
            };

            timer.start();
        }

    }

    private void updateCards()
    {
        operand1.setImageResource(getCardResId(op1.getSuit(), op1.getValue()));
        operand2.setImageResource(getCardResId(op2.getSuit(), op2.getValue()));
        operand3.setImageResource(getCardResId(op3.getSuit(), op3.getValue()));
        quotient.setImageResource(getCardResId(qu.getSuit(), qu.getValue()));
        setAnswerString("");
        updateAnswer();
        answerDialogText = "Accepted answers:\n";
        if (!trial.getAnswerStrings().isEmpty())
            for (int i=0; i<trial.getAnswerStrings().size(); i++) answerDialogText = answerDialogText + trial.getAnswerStrings().get(i) + "\n";
        else answerDialogText = answerDialogText + "No solution exists.";
    }

    private boolean checkAnswer()
    {
        ArrayList<String> answers = trial.getAnswerStrings();

        //Log.d(TAG, "M: " + "|" + answerString + "|");
        if (answerString == "NP" && answers.isEmpty())
        {
            Toast.makeText(this, "Correct!", Toast.LENGTH_SHORT).show();
            return true;
        }

        for (int i=0; i<answers.size(); i ++) {

            //Log.d(TAG, "C: " + "|" + answers.get(i) + "|");
            if (answerString.equals(answers.get(i))) {
                Toast.makeText(this, "Correct!", Toast.LENGTH_SHORT).show();
                return true;
            }
        }
        Toast.makeText(this, "Try again", Toast.LENGTH_SHORT).show();
        return false;
    }

    private void makeAToast (String toast) { Toast.makeText(this, toast, Toast.LENGTH_SHORT).show(); }

    private void updateAnswer()
    {
        answer.setText(answerString);
    }

    private void toggleEET() { enableEditText = !enableEditText; }
    private boolean getEET() { return enableEditText; }

    private void addToAnswerString(String s) { answerString = answerString + " " + s; }
    private void setAnswerString (String s) { answerString = s; }

    private Operand getOperand (int i)
    {
        switch (i)
        {
            case 1: return op1;
            case 2: return op2;
            case 3: return op3;
            default: return new Operand(new Card('h', 1));
        }
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

    public Trial randomTrialGenerator()
    {
        boolean goodTrial = false;
        int trialNotPossible = 0;
        Trial trial;

        do
        {
            op1 = new Operand(randomCardGenerator());
            op2 = new Operand(randomCardGenerator());
            op3 = new Operand(randomCardGenerator());
            qu = new Quotient(randomCardGenerator());
            trial = new Trial(op1, op2, op3, qu);

            if (trial.getLevel() == 0) trialNotPossible++;      //if trial is not possible

            switch (userDifficulty)
            {
                case 1:     //EASY
                {
                    if (trial.getLevel() > 0)
                    {
                        goodTrial = true;
                        trialNotPossible = 0;
                    }

                    break;
                }
                case 2:     //MEDIUM
                {
                    if (trial.getLevel() >= 0 | trialNotPossible >= 1)
                    {
                        goodTrial = true;
                        trialNotPossible = 0;
                    }
                    break;
                }
                case 3:     //HARD
                {
                    if (trial.getLevel() == 0 | trial.getLevel() >= 2)
                    {
                        goodTrial = true;
                        trialNotPossible = 0;
                    }
                    break;
                }
                default:
                    return null;
            }
        } while (!goodTrial);

        return trial;
    }

    public Card randomCardGenerator ()
    {
        int val = rand.nextInt(13) + 1;
        int suit = rand.nextInt(4);
        char c;

        /* D=0, H=1, S=2, C=3 */
        switch (suit)
        {
            case 0: c = 'd'; break;
            case 1: c = 'h'; break;
            case 2: c = 's'; break;
            case 3: c = 'c'; break;
            default: c = 'n'; break;
        }

        return new Card(c, val);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_game, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case android.R.id.home:
                if (useTimer) timer.cancel();
                finish();
                Intent i = new Intent(this, IntroActivity.class);
                i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                i.putExtra(EXTRA_TIME_TRIALS, useTimer);
                startActivity(i);
                //NavUtils.navigateUpFromSameTask(this);
                return true;
            case R.id.menu_item_home:
                if (useTimer) timer.cancel();
                finish();
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
        if (useTimer | ScoreKeeper.getScoreMode()) timer.cancel();
        finish();
        Intent i = new Intent(GameActivity.this, IntroActivity.class);
        i.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
        i.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
        i.putExtra(EXTRA_TIME_TRIALS, useTimer);
        startActivity(i);
    }

    @Override
    public void onStop()
    {
        super.onStop();
        if (useTimer | ScoreKeeper.getScoreMode()) timer.cancel();
        //if (useTimer) timer.cancel();
        //finish();


    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        Log.i(TAG, "onSaveInstanceState");
        savedInstanceState.putInt(EXTRA_USER_DIFFICULTY, userDifficulty);
        savedInstanceState.putBoolean(EXTRA_SHOW_ANSWERS, showAnswers);
        savedInstanceState.putString(EXTRA_ANSWER_STRING, answerString);
        savedInstanceState.putBoolean(EXTRA_TIME_TRIALS, useTimer);
        MyParcelable parceable = new MyParcelable(op1.getSuit(), op2.getSuit(), op3.getSuit(), qu.getSuit(), op1.getValue(), op2.getValue(), op3.getValue(), qu.getValue());
        savedInstanceState.putParcelable(EXTRA_TRIAL_PARCEL, parceable);
        savedInstanceState.putLong(EXTRA_TIME, time);
        if (ScoreKeeper.getScoreMode()) savedInstanceState.putInt(EXTRA_SCORED_ROUND_TIME, scoredRoundTime);
    }

    @Override
    public void onRestart()
    {
        super.onRestart();

        if (useTimer) {
            timer = new CountDownTimer(time, 1000) {
                @Override
                public void onTick(long millisUntilFinished) {
                    time = millisUntilFinished;
                    timerText.setText(Integer.toString((int) (millisUntilFinished / 1000) - 1));
                }

                @Override
                public void onFinish() {
                    timerText.setText(Integer.toString(0));
                    makeAToast("Time's up!");
                    if (showAnswers & !ScoreKeeper.getScoreMode())
                    //if (showAnswers)
                    {
                        Intent intent = new Intent(GameActivity.this, AnswerActivity.class);
                        intent.putExtra(EXTRA_ANSWER_STRING, answerDialogText);
                        intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                        intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                        intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                        intent.putExtra(EXTRA_SUITS, new char[]{op1.getSuit(), op2.getSuit(), op3.getSuit(), qu.getSuit()});
                        intent.putExtra(EXTRA_VALUES, new int[]{op1.getValue(), op2.getValue(), op3.getValue(), qu.getValue()});
                        startActivity(intent);
                    }
                    else {
                        if (!ScoreKeeper.getScoreMode())
                        {
                            timerText.setText(Integer.toString(0));
                            trial = randomTrialGenerator();
                            updateCards();
                            timer.cancel();
                            timer.start();
                        }
                        else
                        {
                            //add score to top 10
                            ScoreKeeper.endRound();
                            boolean isInTop10 = ScoreKeeper.addToTop10();
                            if (isInTop10)
                            {
                                Intent intent = new Intent(GameActivity.this, EnterNameTop10Activity.class);
                                intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                                intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                                intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                                startActivity(intent);
                            }
                            else
                            {
                                Intent intent = new Intent(GameActivity.this, Top10ScoresActivity.class);
                                intent.putExtra(EXTRA_USER_DIFFICULTY, userDifficulty);
                                intent.putExtra(EXTRA_SHOW_ANSWERS, showAnswers);
                                intent.putExtra(EXTRA_TIME_TRIALS, useTimer);
                                startActivity(intent);
                            }
                        }
                    }
                }
            };

            timer.start();
        }
    }


}
