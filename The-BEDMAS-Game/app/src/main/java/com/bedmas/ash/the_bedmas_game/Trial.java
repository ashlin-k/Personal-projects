package com.bedmas.ash.the_bedmas_game;
import android.util.Log;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by Ash on 2015-08-07.
 */
public class Trial {

    private Operand a, b, c;
    private Quotient q;
    private boolean isSolveable;
    private int numWaysSolve;
    private int level; /* NP=0, EASY=1, MED=2, HARD=3, INVALID=-1 */
    private ArrayList<String> answerStrings;

    private final String TAG = "Trial";

    public Trial (Operand c1, Operand c2, Operand c3, Quotient c4)
    {
        a = c1;
        b = c2;
        c = c3;
        q = c4;
        isSolveable = false;
        numWaysSolve = -1;
        level = -1;
        answerStrings = new ArrayList<String>();

        applyBedmas();
        determineLevel();

    }

    public void determineLevel ()
    {
        if (numWaysSolve > 4) level = 1;
        else if (numWaysSolve > 1 && numWaysSolve <= 4) level = 2;
        else if (numWaysSolve == 1) level = 3;
        else if (numWaysSolve == 0) level = 0;
        else level = -1;    //ensures applyBedmas() is called first

    }

    public int getLevel() { return level; }

    public ArrayList<String> getAnswerStrings() { return answerStrings; }

    void applyBedmas ()
    {
        numWaysSolve = 0;

        isSolveable = subtractionSolve(a,b,c);
        if (!isSolveable) isSolveable = subtractionSolve(a, c, b);
        else subtractionSolve(a, c, b);
        if (!isSolveable) isSolveable = subtractionSolve(b, c, a);
        else subtractionSolve(b, c, a);
        if (!isSolveable) isSolveable = subtractionSolve(b, a, c);
        else subtractionSolve(b, a, c);
        if (!isSolveable) isSolveable = subtractionSolve(c, a, b);
        else subtractionSolve(c, a, b);
        if (!isSolveable) isSolveable = subtractionSolve(c, b,a);
        else subtractionSolve(c, b, a);

        if (!isSolveable) isSolveable = additionSolve(a, b, c);
        else additionSolve(a, b, c);
        if (!isSolveable) isSolveable = additionSolve(a, c, b);
        else additionSolve(a, c, b);
        if (!isSolveable) isSolveable = additionSolve(b, c, a);
        else additionSolve(b, c, a);
        if (!isSolveable) isSolveable = additionSolve(b, a, c);
        else additionSolve(b, a, c);
        if (!isSolveable) isSolveable = additionSolve(c, a, b);
        else additionSolve(c, a, b);
        if (!isSolveable) isSolveable = additionSolve(c,b,a);
        else additionSolve(c,b,a);

        if (!isSolveable) isSolveable = multiplicationSolve(a, b, c);
        else multiplicationSolve(a, b, c);
        if (!isSolveable) isSolveable = multiplicationSolve(a, c, b);
        else multiplicationSolve(a, c, b);
        if (!isSolveable) isSolveable = multiplicationSolve(b, c, a);
        else multiplicationSolve(b, c, a);
        if (!isSolveable) isSolveable = multiplicationSolve(b, a, c);
        else multiplicationSolve(b, a, c);
        if (!isSolveable) isSolveable = multiplicationSolve(c, a, b);
        else multiplicationSolve(c, a, b);
        if (!isSolveable) isSolveable = multiplicationSolve(c,b,a);
        else multiplicationSolve(c,b,a);

        if (!isSolveable) isSolveable = divisionSolve(a, b, c);
        else divisionSolve(a, b, c);
        if (!isSolveable) isSolveable = divisionSolve(a, c, b);
        else divisionSolve(a, c, b);
        if (!isSolveable) isSolveable = divisionSolve(b, c, a);
        else divisionSolve(b, c, a);
        if (!isSolveable) isSolveable = divisionSolve(b, a, c);
        else divisionSolve(b, a, c);
        if (!isSolveable) isSolveable = divisionSolve(c, a, b);
        else divisionSolve(c, a, b);
        if (!isSolveable) isSolveable = divisionSolve(c,b,a);
        else divisionSolve(c,b,a);

        if (!isSolveable) isSolveable = exponentSolve(a, b, c);
        else exponentSolve(a, b, c);
        if (!isSolveable) isSolveable = exponentSolve(a, c, b);
        else exponentSolve(a, c, b);
        if (!isSolveable) isSolveable = exponentSolve(b, c, a);
        else exponentSolve(b, c, a);
        if (!isSolveable) isSolveable = exponentSolve(b, a, c);
        else exponentSolve(b, a, c);
        if (!isSolveable) isSolveable = exponentSolve(c, a, b);
        else exponentSolve(c, a, b);
        if (!isSolveable) isSolveable = exponentSolve(c, b, a);
        else exponentSolve(c, b, a);
    }

    boolean subtractionSolve(Operand a, Operand b, Operand c)
    {
        boolean solved = false;

        if ((a.getValue() - b.getValue() - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() - b.getValue() + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() - b.getValue() * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() - (int)((double)b.getValue() / (double)c.getValue() + 0.5)) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() - Math.pow(b.getValue(), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }
        if (((a.getValue() - b.getValue()) * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " ) * " + Integer.toString(c.getValue()));
        }
        if ((int)((double)(a.getValue() - b.getValue()) / (double)c.getValue() + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " ) / " + Integer.toString(c.getValue()));
        }
        if ((Math.pow((a.getValue() - b.getValue()), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " - " + Integer.toString(b.getValue()) + " ) ^ " + Integer.toString(c.getValue()));
        }

        return solved;
    }

    boolean additionSolve (Operand a, Operand b, Operand c)
    {
        boolean solved = false;

        if ((a.getValue() + b.getValue() - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() + b.getValue() + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() + b.getValue() * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() + (int)((double)b.getValue() / (double)c.getValue() + 0.5)) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() + Math.pow(b.getValue(), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }
        if (((a.getValue() + b.getValue()) * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " ) * " + Integer.toString(c.getValue()));
        }
        if ((int)((double)(a.getValue() + b.getValue()) / (double)c.getValue() + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " ) / " + Integer.toString(c.getValue()));
        }
        if ((Math.pow((a.getValue() + b.getValue()), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " + " + Integer.toString(b.getValue()) + " ) ^ " + Integer.toString(c.getValue()));
        }

        return solved;
    }

    boolean multiplicationSolve (Operand a, Operand b, Operand c)
    {
        boolean solved = false;

        if ((a.getValue() * b.getValue() - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() * b.getValue() + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() * b.getValue() * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() * (int)((double)b.getValue() / (double)c.getValue() + 0.5)) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() * Math.pow(b.getValue(), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }
        if ((a.getValue() * (b.getValue() - c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * ( " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()) + " )");
        }
        if ((a.getValue() * (b.getValue() + c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " * ( " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()) + " )");
        }
        if ((int)((double)(a.getValue() * b.getValue()) / (double)c.getValue() + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " ) / " + Integer.toString(c.getValue()));
        }
        if ((Math.pow((a.getValue() * b.getValue()), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " * " + Integer.toString(b.getValue()) + " ) ^ " + Integer.toString(c.getValue()));
        }

        return solved;
    }

    boolean divisionSolve (Operand a, Operand b, Operand c)
    {
        boolean solved = false;

        if (((int)((double)a.getValue() / (double)b.getValue() + 0.5) - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()));
        }
        if (((int)((double)a.getValue() / (double)b.getValue() + 0.5) + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()));
        }
        if (((int)((double)a.getValue() / (double)b.getValue() + 0.5) * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()));
        }
        if ((int)((int)((double)a.getValue() / (double)b.getValue() + 0.5) / c.getValue() + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()));
        }
        if ((int)((double)a.getValue() / Math.pow(b.getValue(), c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }
        if (b.getValue() != c.getValue()) if ((int)((double)a.getValue() / (double)(b.getValue() - c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()) + " )");
        }
        if ((int)((double)a.getValue() / (double)(b.getValue() + c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()) + " )");
        }
        if ((int)((double)a.getValue() / (double)(b.getValue()  * c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()) + " )");
        }
        if ((Math.pow((int)((double)a.getValue() / (double)b.getValue() + 0.5), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" ( " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " ) ^ " + Integer.toString(c.getValue()));
        }
        if(( (double)a.getValue() / (double)(b.getValue() - c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()) + " )");
        }
        if(( (double)a.getValue() / (double)(b.getValue() + c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()) + " )");
        }
        if(( (double)a.getValue() / (double)(b.getValue() * c.getValue()) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()) + " )");
        }
        if(( (double)a.getValue() / (double)((int)((double)b.getValue() / (double)c.getValue() + 0.5)) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / ( " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()) + " )");
        }
        if(( (double)a.getValue() / (double)((int)Math.pow(b.getValue(), c.getValue())) + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " / " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }

        return solved;
    }

    boolean exponentSolve (Operand a, Operand b, Operand c)
    {
        boolean solved = false;

        if ((Math.pow(a.getValue(), b.getValue()) - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()));
        }
        if ((Math.pow(a.getValue(), b.getValue()) + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()));
        }
        if ((Math.pow(a.getValue(), b.getValue()) * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()));
        }
        if ((int)(Math.pow(a.getValue(), b.getValue()) / (double)c.getValue() + 0.5) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()));
        }
        if (Math.pow(a.getValue(), Math.pow(b.getValue(), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()));
        }
        if (Math.pow(a.getValue(), b.getValue() - c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ ( " + Integer.toString(b.getValue()) + " - " + Integer.toString(c.getValue()) + " )");
        }
        if (Math.pow(a.getValue(), b.getValue() + c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ ( " + Integer.toString(b.getValue()) + " + " + Integer.toString(c.getValue()) + " )");
        }
        if (Math.pow(a.getValue(), b.getValue() * c.getValue()) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ ( " + Integer.toString(b.getValue()) + " * " + Integer.toString(c.getValue()) + " )");
        }
        if (Math.pow(a.getValue(), (int)((double)b.getValue() / (double)c.getValue() + 0.5)) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ ( " + Integer.toString(b.getValue()) + " / " + Integer.toString(c.getValue()) + " )");
        }
        if (Math.pow(a.getValue(), Math.pow(b.getValue(), c.getValue())) == q.getValue())
        {
            numWaysSolve++;
            solved = true;
            answerStrings.add(" " + Integer.toString(a.getValue()) + " ^ ( " + Integer.toString(b.getValue()) + " ^ " + Integer.toString(c.getValue()) + " )");
        }

        return solved;
    }
}
