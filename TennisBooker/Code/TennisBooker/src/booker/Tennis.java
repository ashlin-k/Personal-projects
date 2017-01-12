package booker;

import java.io.File;
import java.util.Calendar;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import org.openqa.selenium.By;
import org.openqa.selenium.JavascriptExecutor;
import org.openqa.selenium.Keys;
import org.openqa.selenium.NoSuchElementException;
import org.openqa.selenium.StaleElementReferenceException;
import org.openqa.selenium.WebDriver;
import org.openqa.selenium.WebDriverException;
import org.openqa.selenium.WebElement;
import org.openqa.selenium.chrome.ChromeDriver;
import org.openqa.selenium.firefox.FirefoxBinary;
import org.openqa.selenium.firefox.FirefoxDriver;
import org.openqa.selenium.firefox.FirefoxProfile;
import org.openqa.selenium.interactions.Actions;
import org.openqa.selenium.remote.UnreachableBrowserException;

/* this program needs to run every monday, tuesday, thursday and saturday morning at 730pm
 * to book the following week's court
 * log in at 7:28
 * at 7:30 select day from the table page
 * then book the following times: 
 * 		tuesday 4pm, court 4
 * 		thursday 4pm, 5pm, court 4  
 * 		saturday 1030am, court 4
 * 		the following monday 5pm, 6pm, any 4 courts
 * alternate courts: any other
 * alternate times: none
 * 
 * on saturday, book court under Christie, Jason
 * all other days, book court under O'Connor, David
 * 
 * 4pm is tr 21 from a 1 index
 * 5pm is tr 23 from a 1 index
 * 6pm is tr 25 from a 1 index
 * 1030am is tr 10 from a 1 index
*/

public class Tennis {
	
	public Tennis () { }
	
	public void bookCourt(int time, int court, String secondPlayer, int hour, int minute, int second, int millisec) {
		
		// make strings for xpath
		//xString cell = "//*[@id='MainContent_DayPilotCalendar1']/div/div/table/tbody/tr/td/div/table/tbody/tr[" + time + "]/td[" + court + "]/div";
		String cell = "//*[@id='MainContent_DayPilotCalendar1']/div[2]/div/table/tbody/tr/td[2]/div/table[1]/tbody/tr[" + time + "]/td[" + court + "]/div[1]";

		// FIREFOX
		//WebDriver driver = new FirefoxDriver();
		// CHROME
		System.setProperty("webdriver.chrome.driver", "C:\\Selenium\\chromedriver.exe");
		WebDriver driver = new ChromeDriver();
		// Wait For Page To Load
		// Put a Implicit wait, this means that any search for elements on the
		// page
		// could take the time the implicit wait is set for before throwing
		// exception
		driver.manage().timeouts().implicitlyWait(60, TimeUnit.SECONDS);
		// Navigate to URL
		driver.get("https://inmotion.graniteclub.com/Default.aspx?ReturnUrl=%2f");
		// Maximize the window.
		driver.manage().window().maximize();
		// Enter UserName
		driver.findElement(By.id("MainContent_Login1_UserName")).sendKeys("KY102");
		// Enter Password
		driver.findElement(By.id("MainContent_Login1_Password")).sendKeys("2668");
		// Wait For Page To Load
		driver.manage().timeouts().implicitlyWait(60, TimeUnit.SECONDS);
		// Click on 'Log In' button
		driver.findElement(By.id("MainContent_Login1_LoginLinkButton")).click();
		// Click on COURTS button
		driver.findElement(By.cssSelector("a[href*='BookingsByDay.aspx']")).click();
		
		// Create a timer to click the day button
		Timer timer = new Timer();
		Calendar cal = Calendar.getInstance();
		cal.set(Calendar.HOUR_OF_DAY,hour);
		cal.set(Calendar.MINUTE, minute);
		cal.set(Calendar.SECOND, second);
		cal.set(Calendar.MILLISECOND,millisec);
		Date date = cal.getTime();
		System.out.println(cal);
		timer.schedule(new TimerTask() {
			
			@Override
			public void run() {
				
				boolean pageInvalid = true;
				int count = 0;
				
				driver.findElement(By.id("MainContent_dlDates_btn_6")).click();
				timeDelay(3);
				WebElement cellElement = driver.findElement(By.xpath(cell));
				
				while (pageInvalid)
				{
					try {						
						// Click cell
						int x = cellElement.getLocation().x;
						int y = cellElement.getLocation().y;
						System.out.format("X: %d\tY: %d\n", x, y);
						
						// FOR CHROME
						Actions builder = new Actions(driver);   
						builder.moveToElement(cellElement, 0, 0).click().perform();
						
						// FOR FIREFOX
						//driver.findElement(By.xpath(cell)).click();
						
						driver.manage().timeouts().implicitlyWait(60, TimeUnit.SECONDS);
						timeDelay(1);
						// enter second player's name
						WebElement input = driver.findElement(By.id("MainContent_txtName"));
						input.sendKeys(secondPlayer);
						// click 'Confirm player'
						driver.findElement(By.id("MainContent_btnAddGuest")).click();
						timeDelay(1);
						// click confirm booking
						driver.findElement(By.id("MainContent_btnConfirm")).click();
						pageInvalid = false;						

						System.out.println("Booking complete");
					} 
					// clicked button before 7:30, must navigate back and retry
					catch (NoSuchElementException e) {
						System.out.println("Cell does not exist on this page. Will attempt to re-access.");
						// navigate back a page
						driver.navigate().back();
						timeDelay(1);
						driver.manage().timeouts().implicitlyWait(60, TimeUnit.SECONDS);
						// navigate to correct page/date
						driver.findElement(By.cssSelector("a[href*='BookingsByDay.aspx']")).click();
						timeDelay(1);
						driver.findElement(By.id("MainContent_dlDates_btn_6")).click();
						timeDelay(1);
					}
					// browser has been shut down
					catch (UnreachableBrowserException e) {
						System.out.println("Browser has closed.");
						pageInvalid = false;
					}
					// element is not clickable or unavailable (may be accessed by someone else)
					catch (StaleElementReferenceException e) {
						System.out.println("Element cannot be accessed. Will attempt to re-access.");
						if (count >= 50) pageInvalid = false;
						count++;
					}
					// element is not clickable at point (x,y)
//					catch (WebDriverException e)
//					{
//						System.out.println("Selenium not clicking on the correct coordinates.");
//				        String message = getRootCause(e).getMessage();
//				        System.out.format("######### MESSAGE ############\n%s\n#################\n", message);
//				        String[] arr = message.split(" ");
//				        if ((arr[0] == "Element") && (arr[1] == "is") && (arr[2] == "not") &&
//				        		(arr[3] == "clickable") && (arr[4] == "at") && (arr[5] == "point"))
//				        {
//				        	int xcoord = (int)Double.parseDouble(arr[6].replace("(", "").replace(",", ""));
//				        	int ycoord = (int)Double.parseDouble(arr[7].replace(").", ""));
//				        	System.out.format("Xcoord = %d\tYcoord = %d\n", xcoord, ycoord);
//				        }				       
//					}
				}
				
				// Close the browser.
				timeDelay(2);
				//driver.close();	
				driver.quit();
			}
			
		}, date);
		
	}
	
	private void timeDelay (int timeInSec) {
		try {
			TimeUnit.SECONDS.sleep(timeInSec);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public static Throwable getRootCause(Throwable throwable) {
	    if (throwable.getCause() != null)
	        return getRootCause(throwable.getCause());
	    return throwable;
	}

}
