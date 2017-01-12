package com.bedmas.ash.the_bedmas_game;

/**
 * Created by Ash on 2015-08-07.
 */
public class Card {

    private char suit;
    private int value;

    public Card (char s, int v)
    {
        suit = s;   /* D=0, H=1, S=2, C=3 */
        value = v;
    }

    public char getSuit() {
        return suit;
    }

    public int getValue() {
        return value;
    }
}
