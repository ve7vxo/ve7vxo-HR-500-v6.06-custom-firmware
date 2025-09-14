# HR-500-v6.06-custom-firmware

Control firmware for HR-500 RF amplifier

\*\* Disclaimer: This is experimental code. Use at your own risk. The author of this code assumes no liability for its use.\*\*



Modified firmware for Hardrock HR-500 RF amplifier.

Major re-write of original code. New features, bug fixes.
Scrapped port change interrupt code and wrote polled code. Removed 1ms timer overflow interrupt which was dominating system realtime. Watchdog implemented.

NEWS: YO6SSW has re-written drivers for the display board to reduce latency. This should improve the bargraph display dynamics substantially.

Version 5.00 Notes. -- VE7VXO

-- Changes colour scheme on displays. -- A peak indicator on the bargraph is added. -- Band display text colour changes to match antenna display text colour as a reminder of antenna selection per band as defined in Set\_Theme() colour definitions.

-- 57600 and 115200 Baud added to AccBaud menu. -- Default AccBaud set to 38400. -- Default USB Baud set to 115200.

-- Modem added to XCVR menu. -- Default Band set to 160m. -- Incorporated fault latching ideas from Geert Jan deGroot PE1HZG. Trip condition needs mode press to unlatch.

-- There was a bug in some of the display calculations which had a map() function nested inside a constrain() which is a faux-pas for the arduino compiler. Re-wrote these sections with a simple map(). -- Moving average filters replaced with exponential weighted moving average which is easier to tune.

-- All attempts to make the bar graph lively and dynamic in previous versions met with failure.
The TFT display hardware is just too slow to draw a full bar so the previous code's while loop was eliminated and new code written to only draw a four pixel wide vertical line like the peak indicator.
This works very well. :^>

-- Code edits from PE1HZG to put F/C temperature unit selection in menu system and use of snprintf statements for menu items incorporated in this version. Thank you Geert Jan!

-- Improvements made to VSWR meter routines to improve display dynamic. -- T/R switching routine simplified and moved to top of main loop since many other routines are dependent on TX/RX state. -- Some other niceties like having swr display blank until a valid reading, resetting swr display after changing antennas. Hold swr and fault displays after fault trip. Last vswr appears after drive power removed following a trip.

-- Changed colour references to use different colour variable names which are defined in this header file.

-- Menu item code taken from PE1HZG for temperature unit selection.

-- HRTS command (Temperature units Set) added to serial routines for reading/setting display theme. F = Fahrenheit C = Celsius. Thanks to Geert Jan De Groot PE1HZG.

-- Added menu item for theme selection. Currently only two themes supported NORMAL and NIGHT.
Colour definition can be changed in Set\_Theme() routine. Used 16 bit colour picker from https://rgbcolorpicker.com/565

-- HRDS command (Display Set) added to serial routines for reading/setting display theme. D = day N = night.

-- This version does not use the port change interrupt for PTT sensing but polls the port bit in the main loop and uses a Chronometer for debouncing. Previously there was an occasional glitch if PTT changed while interrupts were disabled. Debounce time is defined in this header file.

-- intSensor() routine modified to be dependent on minimal drive power sensed. This keeps the exponential filters from de-integrating during CW characters and gives more accurate and smoother power display bargraph.

-- Moved carrier detect routines into top of main loop alogside PTT detection and added chronometer for T/R delay to prevent relay wear during fast CW keying with or without PTT signal.

-- Found out needless timer interrupts every millisecond were preventing the freqcount library from operating properly so removed timer interrupt completely.


Version 6.00 Notes. -- VE7VXO

-- Added array in memory and in EEPROM for ATU state per band. Amplifier now remembers if ATU is ON/BYP per band.

-- Implemented watchdog timer to reset amp in case of stack overflow or other surprise endings.

-- Changed TX indicator to reflect PTT vs COR activation based on colour

-- Mode text and colour changed.


Version 6.01 Notes. -- VE7VXO

-- Fault reset bug fixed.


Version 6.02 Notes. -- VE7VXO

-- Small change to touch screen detection.


Version 6.03 Notes. -- VE7VXO

-- Minor display tweaks.
-- New subroutine to read power regs from LTC2945
-- Hysteresis added to SWR display


Version 6.04 Notes. -- VE7VXO

-- New subroutine sets max DC power threshold registers in LTC2945 power monitor IC
-- and enables max power fault bit in control reg.
-- Max DC power is defined on line 27 on HR\_500\_V6.04.ino file (default 1000 watts)

-- DC overpower fault will trip the amp and require drive power to be removed before
-- the amp will reset.  The condition is indicated by red letters "OVER" displayed in
-- DC power display window.  The LTC2945 sample rate is 7.5 Hz so it can interrupt
-- the amplifier Vdd supply in ~133msec.

-- This feature can be used to prevent the operator from going beyond legal limits
-- for example in Canada advanced operators are limited to 1000 watts (DC input) for
-- continuous wave emissions or 700W rf output whichever comes first.
-- Adjust this limit upwards at your own peril.



Version 6.05 Notes. -- VE7VXO

-- Some display tweaks and bug fixes.
-- Improved auto band switching based on frequency detect (amp must be in OFF mode)
-- Remembers last band in use at powerdown and returns to that band on powerup.
-- Tried to speed up RF sensing to improve SWR display with fast CW keying.
-- Some small bugs with ATU display functions not working correctly found and fixed.
-- Bug introduced between Ver.5 and Ver.6 due to inadvertent deletion of instruction
   to drive I/O expander IC chip select line on LPF board. Fixed.
-- T/R routines adjusted to make TX active "LED" panel on right touch screen show
   activity when amp is OFF.

 Version 6.06 Notes. -- VE7VXO

  -- Serial routines for firmware update modified as per PE1HZG to mitigate HobbyPCB
     firmware upload tool connection issues on some windows PC's. 
  -- Eliminated relay chatter due to CW keying when amp is OFF.
  -- Moved Max DC Power define statement (default 1000W) to HR500V1.h line 146.

    

\*/

