package com.bedmas.ash.the_bedmas_game;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.CountDownTimer;
import android.util.Log;

/**
 * Created by Ash on 2015-08-25.
 */
public class ScoreKeeper {

    private static ScoreKeeper sScoreKeeper;
    private static SharedPreferences spScores, spPlayers;
    private static int[] top10Scores = {0,0,0,0,0,0,0,0,0,0};
    private static String[] top10Players = {"-", "-", "-", "-", "-","-", "-", "-", "-", "-"};
    private static boolean scoreMode, roundIsScoring = false;
    private static int scoreCurrentRound = 0, scoreLastRound = 0, index = -1, previousTime = 0, roundTimeSec = 0;
    private static Context sContext;
    private static final String SK_TAG = "ScoreKeeper";
    private static final String MyScores = "com.bedmas.ash.the_bedmas_game.MY_SCORES";
    private static final String MyPlayers = "com.bedmas.ash.the_bedmas_game.MY_PLAYERS";

    private ScoreKeeper (Context c)
    {
        sContext = c;
    }

    static ScoreKeeper get(Context c)
    {
        if (sScoreKeeper == null)
        {
            sScoreKeeper = new ScoreKeeper(c.getApplicationContext());
        }

        int[] jj = top10Scores;
        String[] kk = top10Players;

        String[] keys = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};
        spScores = c.getSharedPreferences(MyScores, Context.MODE_PRIVATE);
        spPlayers = c.getSharedPreferences(MyPlayers, Context.MODE_PRIVATE);
        try {
            for (int i = 0; i < 10; i++) {
                top10Scores[i] = spScores.getInt(keys[i], 0);
                top10Players[i] = spPlayers.getString(keys[i], "-");
            }
        } catch (ClassCastException e) {

            for (int j = 0; j < 10; j++) {
                top10Scores[j] = 0;
                top10Players[j] = "-";
            }
        }

        jj = top10Scores;
        kk = top10Players;

        return sScoreKeeper;
    }

    int[] getTop10Scores () { return top10Scores; }

    static int calculateCurrentScore (long timeRemainingMs, int level)
    {
        if (roundIsScoring & roundTimeSec != 0)
        {
            int timeSeconds = (int) (timeRemainingMs / 1000) - 1;
            int timeRemaining = roundTimeSec - (previousTime - timeSeconds);
            int score = (int) ((double)(timeRemaining * level) / Math.pow((double)roundTimeSec/60.0,2));
            scoreCurrentRound += score;
            previousTime = timeSeconds;
            return score;
        }
        return -1;
    }

    static int getCurrentScore () { return scoreCurrentRound; }

    static int endRound ()
    {
        roundIsScoring = false;
        previousTime = 0;
        roundTimeSec = 0;
        scoreLastRound = scoreCurrentRound;
        scoreCurrentRound = 0;
        return scoreLastRound;
    }

    static void resetRound(int roundtimeMin)
    {
        roundTimeSec = roundtimeMin * 60;
        previousTime = roundtimeMin * 60;
        roundIsScoring = true;
    }

    static void deductPointsForSkip (boolean forSkip)
    {
        if (forSkip) scoreCurrentRound -= 30;
        else scoreCurrentRound -=10;
    }

    static boolean getScoreMode () { return scoreMode; }

    static void setScoreMode(boolean scoremode) { scoreMode = scoremode; }

    static boolean addToTop10 ()
    {
        boolean isInTop10 = false;

        int[] tempArray = new int[10];
        int k = 0;

        //check if current score is in the top 10
        for (int i = 0; i < 10; i++)
        {
            if (scoreLastRound > top10Scores[i] & !isInTop10)
            {
                tempArray[i] = scoreLastRound;
                index = i;
                isInTop10 = true;
            }
            else{
                tempArray[i] = top10Scores[k];
                k++;
            }
        }

        top10Scores = tempArray;

        return isInTop10;
    }

    static void addPlayer(String name)
    {
        String[] temp = {"-", "-", "-", "-", "-","-", "-", "-", "-", "-"};
        int j = 0;
        if (index != -1 & index < 10)
        {
            //top10Players[index] = name;
            for (int i=0; i<10; i++)
            {
                if (i == index) temp[i] = name;
                else
                {
                    temp[i] = top10Players[j];
                    j++;
                }
            }
        }


        top10Players = temp;
        index = -1;

        updateTop10();
    }

    static String generatePlayersString ()
    {
        String scores = "";

        for (int i=0; i<10; i++)
        {
            scores += top10Players[i];
            if (i <9) scores += "\n";
        }

        return scores;
    }

    static String generateScoresString ()
    {
        String scores = "";

        for (int i=0; i<10; i++)
        {
            scores += Integer.toString(top10Scores[i]);
            if (i <9) scores += "\n";
        }

        return scores;
    }

    private static void updateTop10()
    {
        int[] jj = top10Scores;
        String[] kk = top10Players;
        String[] keys = {"1","2","3","4","5","6","7","8","9","10"};
        spScores = sContext.getSharedPreferences(MyScores, Context.MODE_PRIVATE);
        spPlayers = sContext.getSharedPreferences(MyPlayers, Context.MODE_PRIVATE);
        SharedPreferences.Editor editorScores = spScores.edit();
        SharedPreferences.Editor editorPlayers = spPlayers.edit();
        for (int i=0; i<10; i++)
        {
            editorScores.putInt(keys[i], top10Scores[i]);
            editorPlayers.putString(keys[i], top10Players[i]);
        }

        editorScores.commit();
        editorPlayers.commit();
    }
}
