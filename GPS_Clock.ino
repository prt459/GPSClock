// Simple GPS Clock 

#define GPS_RX_PIN 2  // receive serial data pin on the ublox GPS module
#define GPS_TX_PIN 3  // transmit serial data pin on the ublox GPS module

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,10,11,12,13);

SoftwareSerial mySerial(GPS_RX_PIN, GPS_TX_PIN);  // create a Software Serial object

#define ARRAYSIZE 20  // max number of NMEA message params parsed 
String s, token_s, msg_t, msg_tokens[ARRAYSIZE];
int i, j, p=0, cnt, NoSv=0, NoSu=0;
char c = '0', gridsquare[7]="NotSet";
float speed_knts, speed_kmh, speed_ms, alttd;
float lat=0.0, lon=0.0; 

#define OFFSET_GMT  0
#define OFFSET_EST 10  // set the offset for the timezone you live in - Eastern Australian Standard Time
#define OFFSET_DST 11
byte offset_hrs = OFFSET_EST;   // offset (hours) from GMT

unsigned long last_mS;  // for timing the row2 changes  

byte row2 = 0;    // controls the sequence if displays on line 2 of the LCD 

typedef struct poi_type {
  char   poi_name[6];
  char   poi_id[2];
  int    poi_el;
  float  poi_lat;
  float  poi_lon;
};

#define NBR_POI 5
// poi_type poi_table[NBR_POI];  // the table of points of interest
poi_type poi_table[NBR_POI] = {
   (poi_type){"Blkb", " " ,0, -37.826595, 145.151417 },
   (poi_type){"Schl", " ", 0, -37.820181, 145.137547 },
   (poi_type){"MEL ", " ", 0, -37.812969, 144.962929 },
   (poi_type){"SYD ", " ", 0, -33.867719, 151.207462 },
   (poi_type){"LON ", " ", 0,  51.500706, -0.124542 }
};


void calcGridsquare(char *dst, double lat_, double lon_){
// Grid square locators are designated by combination of 2 letters (AA-RR)(a field), 
// the squares by 2 numbers (00-99),and 2 sub-squares letters (aa-xx)). i.e. QF56od 
// is the location Sydney. The algorithm to calculate gridsquare from lat and long 
// was easily found online.

  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;

  dst[0] = (char)0;
  if((lat_==0) || (lon_==00)) return;
  
  // longitude
  remainder = lon_ + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat_ + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)o3 + 'A';
  dst[5] = (char)a3 + 'A';
  dst[6] = (char)0;
};


float calcDistance(float lat_, float lon_){ 
// Distance between the current lat/lon and the lat/lon of a POI.
// Lat and lon are in decimal. 
// Algorithm: https://stackoverflow.com/questions/27928/calculate-distance-between-two-latitude-longitude-points-haversine-formula

  if(lat==0.0 || lon==0.0) return 0.0;
 
  float p = 0.017453292519943295;     // Pi/180
  float a = 0.5 - cos((lat_ - lat) * p)/2 + cos(lat * p) * cos(lat_ * p) * (1 - cos((lon_ - lon) * p)) / 2;
  float distance = 12742 * asin(sqrt(a)); 

  Serial.print("  calcDistance:");  Serial.println(distance,3);
  return distance;
}


int parseNMEA()
{
  // Parses the NMEA message in global s.  Tokens are written to the elements of array msg_tokens (global).
  // Terminates at end of NMEA message ('\n') or after ARRAYSIZE tokens.
  // Returns the number of tokens parsed.  
    int j, n = 0; 
    bool loop_flg = 1;
    String token_s; 
     
    while(loop_flg && (n<ARRAYSIZE)){
      j = s.indexOf(',');
      token_s = s.substring(0, j+1);
//      Serial.print('['); 
      if(s.endsWith("\n")) { token_s = token_s.substring(0, token_s.length()-1); };
//      Serial.print(token_s); 
      msg_tokens[n++] = token_s;  // global array of String tokens
//      Serial.print(']');
      s = s.substring(j+1, s.length());  
      if(j == -1) loop_flg=false;          
    }; 
    // Serial.println(cnt);
//    Serial.println();

    for(j=n; j<ARRAYSIZE; j++) msg_tokens[j] = "";  // empty residual strings    
    return n;
}


void setup()
{
  Serial.begin(9600); 
  while (!Serial) ; // wait for serial port to connect, needed for native USB port only
  mySerial.begin(9600);

  // setup the table of Points Of Interest

  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("GPS Clock - v1.0");
  lcd.setCursor(0,1); 
  lcd.print("github/prt459   ");
  delay(2000);
  lcd.clear(); 
   
  Serial.println("lcd init done");

  pinMode(GPS_RX_PIN, INPUT);
  pinMode(GPS_TX_PIN, OUTPUT);

  last_mS = millis();
}


void loop()
{
  i=0; j=0; c = '0'; s = "";

  while(c != '\n') {
    if (mySerial.available()) {  // check SoftwareSerial port
      c = mySerial.read();       // read the next serial char from the GPS
      s += c;                    // append to message string
    }    
  }
  
  // we have a NMEA message, see what it is...

  msg_t = s.substring(0,6);   // extract the NMEA msg header
//  Serial.println(msg_t);
   
  if(msg_t == "$GPGGA"){
//     Serial.println(s);
     cnt = parseNMEA(); 
     if(cnt >= 9) {
       NoSu = (msg_tokens[7]).toInt();     // satellites used
       alttd = (msg_tokens[9]).toFloat();  // Altitude MSL
     }
  };
              
  if(msg_t == "$GPRMC"){
//     Serial.print(s);
     cnt = parseNMEA();
     Serial.print(msg_tokens[1].substring(0,2));  // hrs
     Serial.print(":"); 
     Serial.print(msg_tokens[1].substring(2,4));  // minutes
     Serial.print(":"); 
     Serial.println(msg_tokens[1].substring(4,6));    // seconds

     String sec_str(msg_tokens[1].substring(4,6));
     byte s = (byte)sec_str.toInt(); 
     if(s%10==0) lcd.clear();
     
     lcd.setCursor(0, 0);
     String hr(msg_tokens[1].substring(0,2));
     byte h = (byte)hr.toInt(); 
     h = (h+offset_hrs)%24;

     lcd.print(h);
     lcd.print(":");
     lcd.print(msg_tokens[1].substring(2,4));
     lcd.print(":");
     lcd.print(msg_tokens[1].substring(4,6));
     switch(offset_hrs){
      case OFFSET_GMT: lcd.print(" GMT"); break;
      case OFFSET_EST: lcd.print(" EST"); break;
      case OFFSET_DST: lcd.print(" DST"); break;
     }

     if(NoSu>0){
       Serial.print("UTC:"); 
       Serial.print(msg_tokens[1].substring(0,2));  // hrs
       Serial.print(":"); 
       Serial.print(msg_tokens[1].substring(2,4));  // minutes
       Serial.print(":"); 
       Serial.print(msg_tokens[1].substring(4));    // seconds

       Serial.print(" "); 
       Serial.print(msg_tokens[9].substring(0,2));  // dd
       Serial.print("/"); 
       Serial.print(msg_tokens[9].substring(2,4));  // mm
       Serial.print("/20"); 
       Serial.print(msg_tokens[9].substring(4));    // yyyy

       Serial.print(" ["); 
       Serial.print(NoSv);  
       Serial.print("] "); 

       lcd.setCursor(13, 0);
       lcd.print("*");
       lcd.print(NoSv);

       if( (s%10==0) || ((millis()-last_mS) > 10000)  ) 
       {
        row2 = (row2+1)%(NBR_POI+2);  
        last_mS = millis();
       }
       
       lcd.setCursor(0, 1);
       if(row2==0)
       {
         // display lat and long on LCD row 2
         Serial.print(" Lat/Long: -"); 
         Serial.print(msg_tokens[3].substring(0,2));  // degrees
         Serial.print(" "); 
         Serial.print(msg_tokens[3].substring(2));  // minutes
//       Serial.print(msg_tokens[4]);  // southing
         Serial.print(", "); 

         lat =  (msg_tokens[3].substring(0,2)).toFloat();  // degrees
         lat += ((msg_tokens[3].substring(2)).toFloat())/60.0;    // minutes and decimal fractions
         lat = lat*(-1); 

         lcd.print(lat, 3);
         lcd.print(" ");

     
         Serial.print(msg_tokens[5].substring(0,3));  // degrees
         Serial.print(" "); 
         Serial.print(msg_tokens[5].substring(3));  // minutes
         Serial.print(" "); 
//       Serial.print(msg_tokens[6]);  // easting

         lon =  (msg_tokens[5].substring(0,3)).toFloat();  // degrees
         lon += ((msg_tokens[5].substring(3)).toFloat())/60;    // minutes and decimal fractions

         lcd.print(lon, 3);
       }
       else if(row2==1)
       { 
         // display speed, altitude and gridsquare on LCD row 2
         speed_knts = msg_tokens[7].toFloat(); 
         speed_kmh = speed_knts*1.852; // 1 knot = 1.852 km/h
         speed_ms = speed_knts*0.514444;  // 1 knot = 0.514444 m/s
     
         Serial.print(" Speed(km/h):"); 
         Serial.print(speed_kmh);   
         Serial.print(" Speed(m/s):"); 
         Serial.print(speed_ms);  

         Serial.print(" Sats used:"); 
         Serial.print(NoSu);  
         Serial.print(" Altitude(m):"); 
         Serial.print(alttd);  

         calcGridsquare(gridsquare, lat, lon);
         Serial.print(" Gridsquare:"); 
         Serial.println(gridsquare);  

         lcd.print(gridsquare);
         lcd.print(" ");
         lcd.print((int)speed_kmh);
         lcd.print("kmh ");
         lcd.print((int)alttd);
         lcd.print("m  ");
       }
       else if(row2 >= 2)
       {
         // display distance to PoI 1 on LCD row 2
         byte p=row2-2; 
         float d = calcDistance(poi_table[p].poi_lat, poi_table[p].poi_lon);
         Serial.print("Distance to "); Serial.print(poi_table[p].poi_name);
         Serial.print("  lat/lon="); Serial.print(lat);Serial.print(" ");Serial.print(lon);
         Serial.print(" "); Serial.print(d);  Serial.println("km ");
         lcd.print(poi_table[p].poi_name);
         lcd.print(":");
         lcd.print(d, 1);
         lcd.print("km    ");
       }
     }
     else
       Serial.println();
//     Serial.println();
//     Serial.print("lat="); 
//     Serial.print(lat, 6); 
//     Serial.print(" lon="); 
//     Serial.print(lon, 6); 
  };

  if(msg_t == "$GPGSV"){
//     Serial.println(s);
     cnt = parseNMEA();
     if((msg_tokens[2] == "2")|| (msg_tokens[2] == "3")){
       NoSv = msg_tokens[3].toInt(); 
     } 
  };

//  Serial.println();

}
