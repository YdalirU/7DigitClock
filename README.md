7Segment Clock

How to scan a 4 Digit display af 7 Segment LEDs to show current time
using an ESP32

Inititally I wanted to see how that works out
Retrieving time from a time server or keep it with an RTC is not that hard
I also added a T/Rh sensor and switched data on the display

For not confusing a somtimes nearly identical display I added an LED
which fires when T/Rh is displayed

The scan process is somewhat tricky though
Too fast and you can't read the digits,  not enigh time to really light up
Too slow and the scanning shows by flickering LEDs


The hard part now is :
When the code retrieves data from the sensor, it takes 21ms which stops the scan
if the process is also covered in the loop(), so the display always flickered

So I put this part of the code to run on core0, independently (RTOS)
I even had to slow the code (Addinng a count variable which allows reading the data)
not a delay() :)

BUT

The ESP32 tends to panic every now and then and I cannot find the bastard.
I added a semaphore so when data is written to the variables in core0 the scanner 
cannot read the data the same time. (I ASSUME this is/was the reason) but 
still the MPU panics.. now and then.. to way to say when and how often

I put this on github for my own good to save and manage the code