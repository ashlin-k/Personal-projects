package com.bedmas.ash.the_bedmas_game;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.ArrayList;

/**
 * Created by Ash on 2015-08-19.
 */
public class MyParcelable implements Parcelable {

    /*
    TRIAL CONSTRUCTOR
    Each Card is make from one char and one int
    private Operand a, b, c;
    private Quotient q;
    private boolean isSolveable;
    private int numWaysSolve;
    private int level;
    private ArrayList<String> answerStrings;
    */

    private char[] suits;
    private int[] values;

    @Override
    public int describeContents() {
        // TODO Auto-generated method stub
        return 0;
    }

    /**
     * Storing the Student data to Parcel object
     **/
    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeCharArray(suits);
        dest.writeIntArray(values);
    }

    /**
     * A constructor that initializes the Student object
     **/
    public MyParcelable(char s1, char s2, char s3, char sq, int v1, int v2, int v3, int vq){
        suits = new char[]{s1, s2, s3, sq};
        values = new int[]{v1, v2, v3, vq};
    }

    // getters and setters

    public char[] getSuits() {
        return suits;
    }

    public void setSuits(char[] suits) {
        this.suits = suits;
    }

    public int[] getValues() {
        return values;
    }

    public void setValues(int[] values) {
        this.values = values;
    }

    /**
     * Retrieving Student data from Parcel object
     * This constructor is invoked by the method createFromParcel(Parcel source) of
     * the object CREATOR
     **/
    private MyParcelable(Parcel in){
        in.readCharArray(suits);
        in.readIntArray(values);
    }

    public static final Parcelable.Creator<MyParcelable> CREATOR = new Parcelable.Creator<MyParcelable>() {

        @Override
        public MyParcelable createFromParcel(Parcel source) {
            return new MyParcelable(source);
        }

        @Override
        public MyParcelable[] newArray(int size) {
            return new MyParcelable[size];
        }
    };


}
