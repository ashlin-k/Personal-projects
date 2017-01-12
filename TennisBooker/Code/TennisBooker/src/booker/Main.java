package booker;

import booker.Tennis;
import booker.TennisThread;

/* TIMES
 * 4pm is tr 21
 * 5pm is tr 23
 * 6pm is tr 25
 * 1030am is tr 10
*/

/* COURTS
 * court 3 is index 2 (currently booked by golf section)
 * court 4 is index 3 (min)
 * court 5 is index 4
 * court 6 is index 5
 * court 7 is index 6
 * goes up to court 12 which is index 11
 */


public class Main {

	public static void main(String[] args) {

		System.out.println("Starting program...");
/*
		String timeString1, courtString1, secondPlayer, hourString, minuteString, secondString, millisecString;
		// Get arguments
		if ((args[0] != null) & (args[1] != null) & (args[2] != null) & (args[4] != null) & (args[4] != null)
				& (args[5] != null) & (args[6] != null)) {
			timeString1 = args[0];
			courtString1 = args[1];
			secondPlayer = args[2];
			hourString = args[3];
			minuteString = args[4];
			secondString = args[5];
			millisecString = args[6];

			// Convert the hour and minute of timer to ints
			int hour = Integer.valueOf(hourString);
			int minute = Integer.valueOf(minuteString);
			int second = Integer.valueOf(secondString);
			int millisec = Integer.valueOf(millisecString);
			if (hour < 0 | hour > 23)
				hour = 7;
			if (minute < 0 | minute > 59)
				minute = 30;
			if (second < 0 | second > 59)
				second = 0;
			if (millisec < 0 | millisec > 999)
				millisec = 0;

			// Convert timestring into an array of ints
			String timeString2 = timeString1.replace("[", "");
			timeString2 = timeString2.replace("]", "");
			String[] timeString3 = timeString2.split(",");
			int[] rowlist = new int[timeString3.length];
			for (int i = 0; i < timeString3.length; i++) {
				switch (timeString3[i]) {
				case "8am":
					rowlist[i] = 5;
					break;
				case "8:30am":
					rowlist[i] = 6;
					break;
				case "9am":
					rowlist[i] = 7;
					break;
				case "9:30am":
					rowlist[i] = 8;
					break;
				case "10am":
					rowlist[i] = 9;
					break;
				case "10:30am":
					rowlist[i] = 10;
					break;
				case "11am":
					rowlist[i] = 11;
					break;
				case "11:30am":
					rowlist[i] = 12;
					break;
				case "12pm":
					rowlist[i] = 13;
					break;
				case "12:30pm":
					rowlist[i] = 14;
					break;
				case "1pm":
					rowlist[i] = 15;
					break;
				case "1:30pm":
					rowlist[i] = 16;
					break;
				case "2pm":
					rowlist[i] = 17;
					break;
				case "2:30pm":
					rowlist[i] = 18;
					break;
				case "3pm":
					rowlist[i] = 19;
					break;
				case "3:30pm":
					rowlist[i] = 20;
					break;
				case "4pm":
					rowlist[i] = 21;
					break;
				case "4:30pm":
					rowlist[i] = 22;
					break;
				case "5pm":
					rowlist[i] = 23;
					break;
				case "5:30pm":
					rowlist[i] = 24;
					break;
				case "6pm":
					rowlist[i] = 25;
					break;
				case "6:30pm":
					rowlist[i] = 26;
					break;
				case "7pm":
					rowlist[i] = 27;
					break;
				case "7:30pm":
					rowlist[i] = 28;
					break;
				case "8pm":
					rowlist[i] = 29;
					break;
				case "8:30pm":
					rowlist[i] = 30;
					break;
				case "9pm":
					rowlist[i] = 31;
					break;
				case "9:30pm":
					rowlist[i] = 32;
					break;
				case "10pm":
					rowlist[i] = 33;
					break;
				case "10:30pm":
					rowlist[i] = 34;
					break;
				default:
					rowlist[i] = 23;
					break;
				}
			}

			// Convert courtstring into an array of ints
			String courtString2 = courtString1.replace("[", "");
			courtString2 = courtString2.replace("]", "");
			String[] courtString3 = courtString2.split(",");
			int[] courtlist = new int[courtString3.length];
			
			// WINTER COURT ALGORITHM
//			for (int i = 0; i < courtString3.length; i++) {
//				courtlist[i] = Integer.valueOf(courtString3[i]) - 3;
//				if (courtlist[i] < 3 | courtlist[i] > 11) courtlist[i] = 4;
//			}
			
			// ALGORITHM ADDED TO ACCOMODATE BUBBLE PROB
			// COURT 4 BOOKINGS ONLY
			//for (int i = 0; i < courtString3.length; i++)
			//{
			//	courtlist[i] = 4-1;
			//}
			
			// SUMMER COURT ALGORITHM
			for (int k=0; k<courtString3.length; k++)
			{
				courtlist[k] = Integer.valueOf(courtString3[k]) + 1;
				if (courtlist[k] > 11) courtlist[k] = 4;
			}
			

			// Ensure number of courts equals the number of times
			if (courtlist.length != rowlist.length)
				return;

			// Create and run TennisThreads
			TennisThread[] threads = new TennisThread[rowlist.length];
			for (int j = 0; j < threads.length; j++) {
				threads[j] = new TennisThread("t" + Integer.toString(j), rowlist[j], courtlist[j], secondPlayer, hour,
						minute, second, millisec);
				System.out.println(
						"Cell - Row: " + Integer.toString(rowlist[j]) + "\tColumn: " + Integer.toString(courtlist[j]));
				threads[j].start();
			}
		}
		 
		 */
		TennisThread t1 = new TennisThread("t1", 6, 4, "O'Connor, David", 21, 29, 0, 0);
		//TennisThread t2 = new TennisThread("t2", 6, 5, "O'Connor, David", 20, 54, 0, 0);
		//TennisThread t3 = new TennisThread("t3", 6, 6, "O'Connor, David", 20, 54, 0, 0);
		
		t1.start();
		//t2.start();
		//t3.start();
		
	}
	
}
