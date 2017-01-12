package com.bedmas.ash.the_bedmas_game;

/**
 * Created by Ash on 2015-08-07.
 */
public class Operand {

    private Card card;

    public Operand (Card c)
    {
        card = c;
    }

    public Card getCard() {
        return card;
    }

    public void setCard(Card card) {
        this.card = card;
    }

    public int getValue() { return card.getValue(); }

    public char getSuit() { return card.getSuit(); }
}
