void getNTP() {
  DEBUG1_PRINTLN("Starting UDP");
  udp.begin(localPort);
  DEBUG3_PRINT("Local port: ");
#ifndef ESP32
  DEBUG3_PRINTLN(udp.localPort());
#endif
  //get a random server from the NTPpool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  // send an NTP request to the time server at the given address
  DEBUG1_PRINTLN("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(timeServerIP, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  // wait to see if a reply is available
  delay(2000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    DEBUG1_PRINTLN("no packet yet");
  }
  else {
    DEBUG1_PRINT("packet received, length=");
    DEBUG1_PRINTLN(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    ntpTime.startup = millis()/1000;
    udp.stop();
    
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    DEBUG3_PRINT("Seconds since Jan 1 1900 = " );
    DEBUG3_PRINTLN(secsSince1900);

    // now convert NTP time into everyday time:
    DEBUG2_PRINT("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // timezone Berlin, ToDo for all
    epoch += 2*3600;
    // print Unix time:
    DEBUG2_PRINTLN(epoch);

    ntpTime.epoch = epoch;
    ntpTime.delta = ntpTime.epoch - ntpTime.startup;
    fillNtpTime(epoch);
    
    // print the hour, minute and second:
    DEBUG1_PRINT("The (UTC) time is ");       // UTC is the time at Greenwich Meridian (GMT)
    DEBUG1_PRINT(ntpTime.mday); 
    DEBUG1_PRINT('.');
    DEBUG1_PRINT(ntpTime.mon); 
    DEBUG1_PRINT('.');
    DEBUG1_PRINT(ntpTime.year); 
    DEBUG1_PRINT(' ');
    DEBUG1_PRINT(ntpTime.hour); // print the hour (86400 equals secs per day)
    DEBUG1_PRINT(':');
    if (ntpTime.min < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      DEBUG1_PRINT('0');
    }
    DEBUG1_PRINT(ntpTime.min); // print the minute (3600 equals secs per minute)
    DEBUG1_PRINT(':');
    if (ntpTime.sec < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      DEBUG1_PRINT('0');
    }
    DEBUG1_PRINTLN(ntpTime.sec); // print the second
  }
}


String fillNtpTime(unsigned long epoch) {
  // Tage seit 1.1.1970 
  unsigned long day = epoch/(24L*60*60);
  /* We now have the number of days since 1970, but need days since 1900 */
  /* There are 17 leap years prior to 1970. Although 1900 was not a leap year */
  /* RTCTIME time cannot represent days before 1970 so make it a leap year to avoid */
  /* the special case (1900 not a leap year, but 2000 is */
  /* 25568 = (70 * 365) + 18 */
  day += 25568;
  // Sekunden am aktuellen Tag
  unsigned long secs = epoch % (24L*60*60);
  ntpTime.sec = secs % 60;
  unsigned long mins = secs / 60;
  ntpTime.hour = mins / 60;
  ntpTime.min = mins % 60;
  ntpTime.wday = (day + 4) % 7;

  /* Calculate year and day within year */
  unsigned long year = (day / (4*365+1)) * 4;
  day %= (4*365+1);
  
  /* If day in year is greater than 365 then adjust so it is 365 or less (0 is Jan 1st) */
  if(day > 365)
  {
      year += ((day - 1) / 365);
      day = (day - 1) % 365;
  }
  
  /* Day of year 1st Jan is 0 */
  ntpTime.yday = day + 1;
  
  /* Not a leap year and is feb 29 or greater */
  /* Then add a day to account for feb has 28, not 29 days */
  /* this is important for the month calculation below */
  ntpTime.leap = (year & 3) && (day >= 59);
  if(ntpTime.leap == 1) {
      day++;
  }
  ntpTime.year = year + 1900;
  
  /* Day of month is what's left, but we want it to start from 1, not 0 */
  // day++;
  
  if (day > 334 + ntpTime.leap) {
    ntpTime.mon = 12;
    day -= 334;
  } else if (day > 304 + ntpTime.leap) {
    ntpTime.mon = 11;
    day -= 304;
  } else if (day > 273 + ntpTime.leap) {
    ntpTime.mon = 10;
    day -= 273;
  } else if (day > 243 + ntpTime.leap) {
    ntpTime.mon =  9;
    day -= 243;
  } else if (day > 212 + ntpTime.leap) {
    ntpTime.mon =  8;
    day -= 212;
  } else if (day > 181 + ntpTime.leap) {
    ntpTime.mon =  7;
    day -= 181;
  } else if (day > 151 + ntpTime.leap) {
    ntpTime.mon =  6;
    day -= 151;
  } else if (day > 120 + ntpTime.leap) {
    ntpTime.mon =  5;
    day -= 120;
  } else if (day >  90 + ntpTime.leap) {
    ntpTime.mon =  4;
    day -= 90;
  } else if (day >  59 + ntpTime.leap) {
    ntpTime.mon =  3;
    day -= 59;
  } else if (day >  31) {
    ntpTime.mon =  2;
    day -= 31;
  } else 
    ntpTime.mon =  1;

  ntpTime.mday = day;
  String result = (ntpTime.mday < 10 ? "0" : "") + String(ntpTime.mday) + "." + (ntpTime.mon < 10 ? "0" : "") + String(ntpTime.mon) + "." + String(ntpTime.year)+ " " + String(ntpTime.hour)+ ":";
  // In the first 10 minutes of each hour, we'll want a leading '0'
  if (ntpTime.min < 10 ) 
    result += "0";
  result += String(ntpTime.min)+ ":";
  if (ntpTime.sec < 10 )
    result += "0";
  result += String(ntpTime.sec);
  return(result);
}
