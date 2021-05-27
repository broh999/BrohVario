/*
Brohvario - developed by Broh999 (Gianluca Selli)

Forked from Mini Vario by Ivaylo Mitev
https://github.com/IvkoPivko/MiniVario-Arduino/tree/master

Bluetooth only and USB powered version.

Features:


Release notes:
Version 1.
*/

#include <Wire.h>
#include <MS5611.h>
MS5611 bpm;



long BaroReadTimeVR = 125;                    // Interval to read the Baro for audio Vario, standard (min) is 150.
long BaroReadTimeBT = 100;                  // Interval to read the baro for BT, standard (min) is 100.

long Pressure, Pressure0, PressureB;

int PinBT,  XOR, c, startCH = 0, Vbat;
float Vario, VarioR, Height, AvrgV, Batt, Temp;

// TimeS is time from start (micros)
unsigned long  dTime, timeE, TimeS, TimePip;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  BaroReadTimeVR = BaroReadTimeVR - 34;
  // BaroReadTimeVR = BaroReadTimeVR - 34; // Whit built in BT.

  Serial.begin(9600);

  pinMode(7, OUTPUT);                     // Pin to the BT supply.
  pinMode(8, OUTPUT);                     // Pin to the BT supply.


  // Initialize MS5611 sensor!
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  
  while (!bpm.begin(MS5611_ULTRA_HIGH_RES))
  {
    delay(500);
  }

  // BT START
  
    digitalWrite(7, HIGH);               // Switch on BT supply.
    digitalWrite(8, HIGH);               // Switch on BT supply.
    delay(1000);
    Serial.begin(9600);                  //for MiniPro
    //Serial1.begin(9600);                 //for BT - Leonardo.
    // On-Off | Here between // remove the * to change the BT name.
      Serial.print("AT");
      delay(1500);
      Serial.print("AT+NAMEBrohVario"); //BT name assigned
      delay(500);
      //Serial.print("AT+RESET");
      delay(500);//*/
    // PIN is 1234 or 0000 <= #################################################################################
  //}
  //else
  //{
  //  digitalWrite(7, LOW);               // Switch off BT supply.
  //  digitalWrite(8, LOW);               // Switch off BT supply.
  //}
  // rename BT END * /

  TimeS = micros();

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP/////////////////////////////////////////////////////////////////////////////////////////////////////


// LOOP///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    Bluetooth();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENDE LOOP//////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=> Sub-functions and programs///   /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Read out the air pressure sensor###############################################################################
// ###############################################################################################################
void BaroReadout()
{
  Temp = bpm.readTemperature();
  Pressure = bpm.readPressure(true);
  Height = bpm.getAltitude(Pressure);
}
// ###############################################################################################################
// ENDE ##########################################################################################################



// Calculate climb## #############################################################################################
// ###############################################################################################################
void CalculateClimb()
{
  BaroReadout();


  int i;

  if (startCH == 0)
  {
    kal[0] = Height;
    startCH = 1;
  }

  // Calculate climb rates.
  dTime = (micros() - TimeS);
  TimeS = micros();

  VarioR = ((Height - kal[0]) / (float(dTime) / 1000000));

  //VarioR=0.500; // Sound test! Comment out during normal operation!
  //kal[1] = VarioR;

  kal[1] = 0.55* VarioR + 0.45* kal[1];  //##############################################

  kal[0] = Height;

  // Apply filter for the slope.
  // > Calculate average value.
  AvrgV = 0;
  i = 1;
  for (i; i <= medium_n; i++) {
    AvrgV = AvrgV + kal[i];
  }
  AvrgV = AvrgV / medium_n;
  AvrgV = (AvrgV  + Vario) / 2;
  // < Calculate average value.

  if (ErrorV > 1.000) ErrorV = 1.000;
  Vario = ErrorV * AvrgV + (1 - ErrorV) * Vario;


  i = medium_n; // mittel is Medium
  for (i; i > 1; i--) {
    kal[i] = kal[i - 1];
  }

        //BT button;dTime[ms];Pressure[Pa];Height[m];VarioR[m/s];Vario[m/s]
        /*/  To activate the output delete * between //
      
        Serial.print(PinBT);
        Serial.print("; ");
      
        Serial.print(float(dTime) / 1000, 2);
        Serial.print("; ");
      
        Serial.print(Pressure);
        Serial.print("; ");
      
        Serial.print(Height, 2);
        Serial.print("; ");
      
        Serial.print(VarioR, 2);
        Serial.print("; ");
      
        Serial.print(Vario, 2);//
        Serial.println(); // */

}
// ###############################################################################################################
// END ##########################################################################################################



// Battery voltage reader ##########################################################################################
// ###############################################################################################################
void AkkuVolt()
{
    Vbat = analogRead(BatV);
	//Batt = 1000.0 + 100.0*(1 - (4.16 - Vbat*(3.30/1023.00)/0.76904762)/0.85);  //  Ist10k/(Ist3k+Ist10k)=0.76904762
    Batt = (Vbat * 5.0/1023.0)*2.0;
    /*  TODO: Voltage alarm: This is needed with LiPo batteries to avoid the total
        discarge
        Temp note
        Using 2S lipo (8.4 max voltage) with 10k+10k voltage divider.
        : Batt = (vbatt * 5.0/1023)*2
    */ 
    if (Batt <= min_batt)
    {
        if ( (millis() - TimePip) >= (unsigned long)(3 * 300) )
            {
            TimePip = millis();
            tone( a_pin1 , 300, 300 );
            }
    }
}
// #############################################################################################################*/ 
// END ##########################################################################################################



// Piepser (BUZZER) ##############################################################################################
// ###############################################################################################################
/*void BuzzerSound()
{
  //Vario = 2.00; // Sound test! Comment out during normal operation!

    float frequency = -0.33332*Vario*Vario*Vario*Vario + 9.54324*Vario*Vario*Vario - 102.64693*Vario*Vario + 512.227*Vario + 84.38465;

    //float duration = 1.6478887*Vario*(Vario/2) -38.2889*Vario + 341.275253; // Variable Pause
    float duration = 300 - (25*Vario);
    frequency = int(frequency);
    duration = long(duration);
  
    // If the climb is greater than the min_climb
    if ( Vario >= min_climb)
    {
        if ( (millis() - TimePip) >= (unsigned long)(1.5 * duration) )
        {
          TimePip = millis();
          tone( a_pin1 , int(frequency), int(duration) );
        }
    }
    // If sink is less than max_sink
    if ( Vario <= max_sink)
    {
      tone(a_pin1 , 300, 150);
      delay(125);
      tone(a_pin1 , 200, 150);
      delay(150);
      tone(a_pin1 , 100, 150);
      delay(175);
    }
}*/
// ###############################################################################################################
// END ##########################################################################################################



// Bluetooth #####################################################################################################
// ###############################################################################################################
/*  Different communication protocols is possible.  */

void Bluetooth()
{


  // Start "Blue Fly Vario" sentence =============================================================================
  // =============================================================================================================
  /* Output in BlueFlyVario format. The standard BlueFlyVario output mode. This sends raw
    pressure measurements in the form "PRS XXXXX\n": XXXXX is the raw (unfiltered) pressure
    measurement in hexadecimal pascals. */

  /*/ On-Off | Put here between // a * then it is deactivated.
  Temp = bpm.readTemperature();
  //Pressure = bpm.readPressure();
  Pressure = 0.250* bpm.readPressure(true) +  0.750* Pressure;

  Serial.print("PRS ");               //Output to the BT for MiniPro.
  Serial.println( Pressure, HEX);        //BT serial interface unfiltered. For MiniPro.

  //Serial1.print("PRS ");               //Issue at the BT for Leonardo.
  //Serial1.println( Pressure, HEX);        //BT serial interface unfiltered. For Leonardo.

  delay(BaroReadTimeBT - 73);

  // If XCSoar is used, comment out the line below with "// ...".
  //delay(BaroReadTimeBT - 22); // If XCTrack is used, leave the line active.

  // End "BlueFlyVario" sentence =========================================================================== */

  // =>>

  // Start "LXNAV - LXWP0" sentence ==============================================================================
  // =============================================================================================================
  /* Send LXWP0 output mode for use with a range of apps:
      "$LXWP0,loger_stored (Y/N), IAS (kph), baroaltitude (m), vario (m/s),,,,,,heading of plane,
      windcourse (deg),windspeed (kph)*checksum \r\n" */
  /*/ On-Off | Put here between // a * then it is deactivated.
      CalculateClimb();

      String s = "LXWP0,N,,";
      s = String(s+ String(Height,1) + "," + String(Vario,2) + ",,,,,,,,"  );

    // Calculate the checksum and output it as an int
    // is required as HEX in the NMEA data set
    // calculate between $ and *
      int i, XOR, c;
      XOR = 0;

      for (i = 0; i < s.length(); i++) {
          c = (unsigned char)s.charAt(i);
          if (c == '*') break;
          if (c!='$') XOR ^= c;
      }
    // Calculate the checksum

      // For MiniPro:
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // For Leonardo:
      //Serial1.print("$");
      //Serial1.print(s);
      //Serial1.print("*");
      //Serial1.println(XOR,HEX); //

    delay(BaroReadTimeBT - 73);

  // End "LXNAV - LXWP0" sentence ========================================================================== */

  // =>>

  // Start "LK8EX1" sentence =====================================================================================
  // =============================================================================================================
  // Send $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  /*
    LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

    Field 0, raw pressure in hPascal:
      hPA*100 (example for 1013.25 becomes  101325) 
      no padding (987.25 becomes 98725, NOT 098725)
      If no pressure available, send 999999 (6 times 9)
      If pressure is available, field 1 altitude will be ignored
    
    Field 1, altitude in meters, relative to QNH 1013.25
      If raw pressure is available, this value will be IGNORED (you can set it to 99999
      but not really needed)! (if you want to use this value, set raw pressure to 999999)
    
    Field 2, vario in cm/s
      If vario not available, send 9999  (4 times 9) Value can also be negative
    
    Field 3, temperature in C , can be also negative
      If not available, send 99
    
    Field 4, battery voltage or charge percentage Cannot be negative
      If not available, send 999 (3 times 9)
      Voltage is sent as float value like: 0.1 1.4 2.3  11.2 
      To send percentage, add 1000. Example 0% = 1000
      14% = 1014 .  Do not send float values for percentages.
    Percentage should be 0 to 100, with no decimals, added by 1000!
  */
  // On-Off | Put here between // a * then it is deactivated. 
    Temp = bpm.readTemperature(true);
    Pressure = 0.250* bpm.readPressure(true) +  0.750* Pressure;
    //CalculateClimb();
    //AkkuVolt();
    
    String s = "LK8EX1,";
    //s = String(s + String(Pressure,DEC) + ",99999,9999," + String(Temp,1) + "," + String(Batt,2) + ",");
    s = String(s + String(Pressure,DEC) + ",99999,9999," + String(Temp,1) + ",");
    // Calculate the checksum and output it as an int
    // is required as HEX in the NMEA data set
    // calculate between $ and *
    int i, XOR, c;
    XOR = 0;

    for (i = 0; i < s.length(); i++) {
    c = (unsigned char)s.charAt(i);
    if (c == '*') break;
    if (c!='$') XOR ^= c;
    }
    // Calculate the checksum

        // For MiniPro:
        Serial.print("$");
        Serial.print(s);
        Serial.print("*");
        Serial.println(XOR,HEX);

        // For Leonardo:
        //Serial1.print("$");
        //Serial1.print(s);
        //Serial1.print("*");
        //Serial1.println(XOR,HEX); // 
    
    delay(BaroReadTimeBT - 30);
  // End "LK8EX1" sentence ================================================================================= */

  // =>>

  // Start "Custom BFV" sentence =================================================================================
  // =============================================================================================================
  /* Custom BFV sentence: This sends a NMEA like sentence in the following format: 

    "$BFV,pressure(Pa),vario(cm/s), temp(deg C), battery(%),pitotDiffPressure(pa)*checksum\r\n"

    Pressure (the filtered pressure as an unsigned integer), vario (the filtered vario as an signed integer) 
    and temp(signed float) are always sent. Battery % (unsigned integer) is only sent for models which include
    a battery; otherwise "0" is sent. pitotDiffPressure (signed integer) is only sent when the hardware setting
    usePitot is enabled. */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.

      //CalculateClimb();
      Temp = bpm.readTemperature(true);
      Pressure = 0.250* bpm.readPressure(true) +  0.750* Pressure;
      
      AkkuVolt();

      String s = "BFV,";
      //s = String(s + String(Pressure,DEC) + "," + String(Vario*100,DEC) + "," + String(Temp,2) + ",");
      s = String(s + String(Pressure,DEC) + ",," + String(Temp,2) + ",");
      s = String(s + String(Batt,DEC) + "," );

    // Checksum berechnen
    // und als int ausgeben wird als HEX benötigt.
    // Im NMEA Datensatz zwischen $ und * rechnen.
      int i, XOR, c;
      XOR = 0;

      for (i = 0; i < s.length(); i++) {
          c = (unsigned char)s.charAt(i);
          if (c == '*') break;
          if (c!='$') XOR ^= c;
      }

      // Fuer MiniPro:
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // Fuer Leonardo:
      //Serial1.print("$");
      //Serial1.print(s);
      //Serial1.print("*");
      //Serial1.println(XOR,HEX); //

    delay(BaroReadTimeBT - 24);

  // End "Custom BFV sentence" ============================================================================= */



  // Start normal data output    =================================================================================
  // =============================================================================================================
  /*/ On-Off | Put here between // a * then it is deactivated.

    // For testing via serial port !!! -> don't forget to comment on VarioR.
    // Temp.[C°];Pressure[Pa];Height[m];dTime[ms];VarioR[m/s];Vario[m/s];BT Taster
    // Activate * between // delete for output.

    CalculateClimb();

    Serial.print(Temp, 2);
    Serial.print("; ");

    Serial.print(Pressure);
    Serial.print("; ");

    Serial.print(Height, 2);
    Serial.print("; ");

    Serial.print(dTime/1000, 3);
    Serial.print("; ");

    Serial.print(VarioR, 2);
    Serial.print("; ");

    Serial.print(Vario, 2);
    Serial.print("; ");

    Serial.print(PinBT);
    Serial.println();

    delay(BaroReadTimeVR - 4);

    // End of normal data output ============================================================================= */
}
// ###############################################################################################################
// END ##########################################################################################################
