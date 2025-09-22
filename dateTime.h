#include <NTPClient.h> /* version 3.2.1 */
#include <WiFiUdp.h> /* version 2.0.0 */

int time_offset = 7200;

// Define an object to manage Wi-Fi connection
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", time_offset, 60000); // "europe.pool.ntp.org" is an NTP server in Europe (Roma UTC+1)


void updateTime(){
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
}

int* getYearMonthDay(){
  int* yearMonthDay = new int[3];
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  yearMonthDay[0] = ptm->tm_year+1900;              //year
  yearMonthDay[1] = ptm->tm_mon+1;                  //month
  yearMonthDay[2] = ptm->tm_mday;                   //day   
  return yearMonthDay;
}

