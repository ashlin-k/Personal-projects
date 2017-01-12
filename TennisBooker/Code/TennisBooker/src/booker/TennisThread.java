package booker;

import booker.Tennis;

public class TennisThread extends Thread {

	private Thread t;
	private String threadName, secondPlayer;
	private int time, court, hour, minute, second, millisec;

	TennisThread(String name, int time, int court, String secondPlayer, int hour, int minute, int second, int millisec) {
		threadName = name;
		System.out.println("Creating " + threadName);
		this.time = time;
		this.court = court;
		this.secondPlayer = secondPlayer;
		this.hour = hour;
		this.minute = minute;
		this.second = second;
		this.millisec = millisec;
	}

	public void run() {
		System.out.println("Running " + threadName);
		Tennis tennis = new Tennis();
		tennis.bookCourt(time, court, secondPlayer, hour, minute, second, millisec);
		System.out.println("Thread " + threadName + " exiting.");
	}

	public void start() {
		System.out.println("Starting " + threadName);
		if (t == null) {
			t = new Thread(this, threadName);
			t.start();
		}
	}

}
