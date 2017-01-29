/*
PV-logger

Copyright (C) 2017, Andrea Marson <am.dev.75@gmail.com>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <Thread.h>
#include <ThreadController.h>
#include <ESP8266.h>
#include <dht.h>
#include <avr/wdt.h>

#define SW_VERSION "0.6.0"

// Enable/disable functionalities
//#define DISABLE_HEARTBEAT
//#define DISABLE_DATA_SAMPLER
//#define DISABLE_WDT
//#define DISABLE_DATA_SAMPLER_TEMP_HUM
//#define DISABLE_DATA_SAMPLER_PV_INVERTER
//#define DISABLE_PV_INVERTER_PROBING

// Debug options
#undef DGB_SERIAL1
//#define DBG_ESP8266
#undef DBG_ESP8266
#define DBG_PV_INVERTER_COMMUNICATION

#define SER_DEBUG_BAUD_RATE         115200

#define SER_ESP8266_BAUD_RATE                   115200
#define WIFI_SSID                               "put your SSID here"
#define WIFI_PASSWD                             "put your password here"
#define SERVER_NAME                             "things.ubidots.com"
#define SERVER_PORT                             (80)
#define UBIDOTS_TOKEN                           "put your token here"
#define UBIDOTS_TEMPERATURE_VAR_ID              "put your ID here"
#define UBIDOTS_HUMIDITY_VAR_ID                 "put your ID here"
#define UBIDOTS_GRID_POWER_VAR_ID               "put your ID here"
#define UBIDOTS_TOTAL_CUMULATED_ENERGY_VAR_ID   "put your ID here"


#define PV_INVERTER_ADDRESS         2
#define RS485_TX_ENABLE_PIN         25
#define RS485_TX_MODE               HIGH
#define RS485_RX_MODE               LOW
#define RS485_SERIAL_TIMEOUT        500
#define RS485_SERIAL_BAUD_RATE      19200

#define DHT_PIN                     12
#define DHT11                       11
#define DHT22                       22
#define DHT_TYPE                    DHT22

#define DATA_SAMPLER_INTERVAL             15000
#define HEART_BEAT_INTERVAL               500
#define WIFI_AP_CONNECTION_TEST_INTERVAL  1000

class cls_PV_inverter {
private:
  int MaxAttempt = 1;
  byte Address = 0;

  void clearData(byte *data, byte len) {
    for (int i = 0; i < len; i++) {
      data[i] = 0;
    }
  }

  int Crc16(byte *data, int offset, int count){
    byte BccLo = 0xFF;
    byte BccHi = 0xFF;

    for (int i = offset; i < (offset + count); i++){
      byte New = data[offset + i] ^ BccLo;
      byte Tmp = New << 4;
      New = Tmp ^ New;
      Tmp = New >> 5;
      BccLo = BccHi;
      BccHi = New ^ Tmp;
      Tmp = New << 3;
      BccLo = BccLo ^ Tmp;
      Tmp = New >> 4;
      BccLo = BccLo ^ Tmp;
    }

    return (int)word(~BccHi, ~BccLo);
  }

  bool Send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, byte param5, byte param6){

    SendStatus = false;
    ReceiveStatus = false;
    int received_bytes_num = 0;

    byte SendData[10];
    SendData[0] = address;
    SendData[1] = param0;
    SendData[2] = param1;
    SendData[3] = param2;
    SendData[4] = param3;
    SendData[5] = param4;
    SendData[6] = param5;
    SendData[7] = param6;

    int crc = Crc16(SendData, 0, 8);
    SendData[8] = lowByte(crc);
    SendData[9] = highByte(crc);

    clearReceiveData();

    for (int i = 0; i < MaxAttempt; i++)
    {
      digitalWrite(RS485_TX_ENABLE_PIN, RS485_TX_MODE);
      delay(50);
#ifdef DBG_PV_INVERTER_COMMUNICATION
	  Serial.println("[PV] Sending message ...");
	  for(int j = 0; j < 9; j++){
		  Serial.print("[PV] TX message [" + String(j) + "] = " + String(SendData[j]) + " = 0x");
      Serial.println(String(SendData[j], HEX));
    }
#endif
      if (Serial1.write(SendData, sizeof(SendData)) != 0) {
        Serial1.flush();
        SendStatus = true;

        digitalWrite(RS485_TX_ENABLE_PIN, RS485_RX_MODE);

        received_bytes_num = Serial1.readBytes(ReceiveData, sizeof(ReceiveData));
        if (received_bytes_num != 0) {
#ifdef DBG_PV_INVERTER_COMMUNICATION
          Serial.println("[PV] RX message length = " + String(received_bytes_num));
          for(int j = 0; j < 8; j++){
            Serial.print("[PV] RX message [" + String(j) + "] = " + String(ReceiveData[j]) + " = 0x");
            Serial.println(String(ReceiveData[j], HEX));
          }
#endif
          if ((int)word(ReceiveData[7], ReceiveData[6]) == Crc16(ReceiveData, 0, 6)) {
#ifdef DBG_PV_INVERTER_COMMUNICATION
            Serial.println("[PV] RX message CRC ok");
#endif
            ReceiveStatus = true;
            break;
          } else {
#ifdef DBG_PV_INVERTER_COMMUNICATION
          Serial.println("[PV] RX message CRC fail");
#endif
          }
        } else {
#ifdef DBG_PV_INVERTER_COMMUNICATION
          Serial.println("[PV] No data received");
#endif
        }
      }
    }
    return ReceiveStatus;
  }

  union {
    byte asBytes[4];
    float asFloat;
  } foo;

  union {
    byte asBytes[4];
    unsigned long asUlong;
  } ulo;

public:
  bool SendStatus = false;
  bool ReceiveStatus = false;
  byte ReceiveData[8];

  cls_PV_inverter(byte address) {
    pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
    digitalWrite(RS485_TX_ENABLE_PIN, RS485_RX_MODE);

    Serial1.setTimeout(RS485_SERIAL_TIMEOUT);
    Serial1.begin(RS485_SERIAL_BAUD_RATE);
    
    Address = address;

    SendStatus = false;
    ReceiveStatus = false;

    clearReceiveData();
  }

  void clearReceiveData() {
    clearData(ReceiveData, 8);
  }

  String TransmissionState(byte id) {
    switch (id)
    {
    case 0:
      return F("Everything is OK.");
      break;
    case 51:
      return F("Command is not implemented");
      break;
    case 52:
      return F("Variable does not exist");
      break;
    case 53:
      return F("Variable value is out of range");
      break;
    case 54:
      return F("EEprom not accessible");
      break;
    case 55:
      return F("Not Toggled Service Mode");
      break;
    case 56:
      return F("Can not send the command to internal micro");
      break;
    case 57:
      return F("Command not Executed");
      break;
    case 58:
      return F("The variable is not available, retry");
      break;
    default:
      return F("Sconosciuto");
      break;
    }
  }

  String GlobalState(byte id) {
    switch (id)
    {
    case 0:
      return F("Sending Parameters");
      break;
    case 1:
      return F("Wait Sun / Grid");
      break;
    case 2:
      return F("Checking Grid");
      break;
    case 3:
      return F("Measuring Riso");
      break;
    case 4:
      return F("DcDc Start");
      break;
    case 5:
      return F("Inverter Start");
      break;
    case 6:
      return F("Run");
      break;
    case 7:
      return F("Recovery");
      break;
    case 8:
      return F("Pausev");
      break;
    case 9:
      return F("Ground Fault");
      break;
    case 10:
      return F("OTH Fault");
      break;
    case 11:
      return F("Address Setting");
      break;
    case 12:
      return F("Self Test");
      break;
    case 13:
      return F("Self Test Fail");
      break;
    case 14:
      return F("Sensor Test + Meas.Riso");
      break;
    case 15:
      return F("Leak Fault");
      break;
    case 16:
      return F("Waiting for manual reset");
      break;
    case 17:
      return F("Internal Error E026");
      break;
    case 18:
      return F("Internal Error E027");
      break;
    case 19:
      return F("Internal Error E028");
      break;
    case 20:
      return F("Internal Error E029");
      break;
    case 21:
      return F("Internal Error E030");
      break;
    case 22:
      return F("Sending Wind Table");
      break;
    case 23:
      return F("Failed Sending table");
      break;
    case 24:
      return F("UTH Fault");
      break;
    case 25:
      return F("Remote OFF");
      break;
    case 26:
      return F("Interlock Fail");
      break;
    case 27:
      return F("Executing Autotest");
      break;
    case 30:
      return F("Waiting Sun");
      break;
    case 31:
      return F("Temperature Fault");
      break;
    case 32:
      return F("Fan Staucked");
      break;
    case 33:
      return F("Int.Com.Fault");
      break;
    case 34:
      return F("Slave Insertion");
      break;
    case 35:
      return F("DC Switch Open");
      break;
    case 36:
      return F("TRAS Switch Open");
      break;
    case 37:
      return F("MASTER Exclusion");
      break;
    case 38:
      return F("Auto Exclusion");
      break;
    case 98:
      return F("Erasing Internal EEprom");
      break;
    case 99:
      return F("Erasing External EEprom");
      break;
    case 100:
      return F("Counting EEprom");
      break;
    case 101:
      return F("Freeze");
    default:
      return F("Sconosciuto");
      break;
    }
  }

  String DcDcState(byte id) {
    switch (id)
    {
    case 0:
      return F("DcDc OFF");
      break;
    case 1:
      return F("Ramp Start");
      break;
    case 2:
      return F("MPPT");
      break;
    case 3:
      return F("Not Used");
      break;
    case 4:
      return F("Input OC");
      break;
    case 5:
      return F("Input UV");
      break;
    case 6:
      return F("Input OV");
      break;
    case 7:
      return F("Input Low");
      break;
    case 8:
      return F("No Parameters");
      break;
    case 9:
      return F("Bulk OV");
      break;
    case 10:
      return F("Communication Error");
      break;
    case 11:
      return F("Ramp Fail");
      break;
    case 12:
      return F("Internal Error");
      break;
    case 13:
      return F("Input mode Error");
      break;
    case 14:
      return F("Ground Fault");
      break;
    case 15:
      return F("Inverter Fail");
      break;
    case 16:
      return F("DcDc IGBT Sat");
      break;
    case 17:
      return F("DcDc ILEAK Fail");
      break;
    case 18:
      return F("DcDc Grid Fail");
      break;
    case 19:
      return F("DcDc Comm.Error");
      break;
    default:
      return F("Sconosciuto");
      break;
    }
  }

  String InverterState(byte id) {
    switch (id)
    {
    case 0:
      return F("Stand By");
      break;
    case 1:
      return F("Checking Grid");
      break;
    case 2:
      return F("Run");
      break;
    case 3:
      return F("Bulk OV");
      break;
    case 4:
      return F("Out OC");
      break;
    case 5:
      return F("IGBT Sat");
      break;
    case 6:
      return F("Bulk UV");
      break;
    case 7:
      return F("Degauss Error");
      break;
    case 8:
      return F("No Parameters");
      break;
    case 9:
      return F("Bulk Low");
      break;
    case 10:
      return F("Grid OV");
      break;
    case 11:
      return F("Communication Error");
      break;
    case 12:
      return F("Degaussing");
      break;
    case 13:
      return F("Starting");
      break;
    case 14:
      return F("Bulk Cap Fail");
      break;
    case 15:
      return F("Leak Fail");
      break;
    case 16:
      return F("DcDc Fail");
      break;
    case 17:
      return F("Ileak Sensor Fail");
      break;
    case 18:
      return F("SelfTest: relay inverter");
      break;
    case 19:
      return F("SelfTest : wait for sensor test");
      break;
    case 20:
      return F("SelfTest : test relay DcDc + sensor");
      break;
    case 21:
      return F("SelfTest : relay inverter fail");
      break;
    case 22:
      return F("SelfTest timeout fail");
      break;
    case 23:
      return F("SelfTest : relay DcDc fail");
      break;
    case 24:
      return F("Self Test 1");
      break;
    case 25:
      return F("Waiting self test start");
      break;
    case 26:
      return F("Dc Injection");
      break;
    case 27:
      return F("Self Test 2");
      break;
    case 28:
      return F("Self Test 3");
      break;
    case 29:
      return F("Self Test 4");
      break;
    case 30:
      return F("Internal Error");
      break;
    case 31:
      return F("Internal Error");
      break;
    case 40:
      return F("Forbidden State");
      break;
    case 41:
      return F("Input UC");
      break;
    case 42:
      return F("Zero Power");
      break;
    case 43:
      return F("Grid Not Present");
      break;
    case 44:
      return F("Waiting Start");
      break;
    case 45:
      return F("MPPT");
      break;
    case 46:
      return F("Grid Fail");
      break;
    case 47:
      return F("Input OC");
      break;
    default:
      return F("Sconosciuto");
      break;
    }
  }

  String AlarmState(byte id) {
    switch (id)
    {
    case 0:
      return F("No Alarm");
      break;
    case 1:
      return F("Sun Low");
      break;
    case 2:
      return F("Input OC");
      break;
    case 3:
      return F("Input UV");
      break;
    case 4:
      return F("Input OV");
      break;
    case 5:
      return F("Sun Low");
      break;
    case 6:
      return F("No Parameters");
      break;
    case 7:
      return F("Bulk OV");
      break;
    case 8:
      return F("Comm.Error");
      break;
    case 9:
      return F("Output OC");
      break;
    case 10:
      return F("IGBT Sat");
      break;
    case 11:
      return F("Bulk UV");
      break;
    case 12:
      return F("Internal error");
      break;
    case 13:
      return F("Grid Fail");
      break;
    case 14:
      return F("Bulk Low");
      break;
    case 15:
      return F("Ramp Fail");
      break;
    case 16:
      return F("Dc / Dc Fail");
      break;
    case 17:
      return F("Wrong Mode");
      break;
    case 18:
      return F("Ground Fault");
      break;
    case 19:
      return F("Over Temp.");
      break;
    case 20:
      return F("Bulk Cap Fail");
      break;
    case 21:
      return F("Inverter Fail");
      break;
    case 22:
      return F("Start Timeout");
      break;
    case 23:
      return F("Ground Fault");
      break;
    case 24:
      return F("Degauss error");
      break;
    case 25:
      return F("Ileak sens.fail");
      break;
    case 26:
      return F("DcDc Fail");
      break;
    case 27:
      return F("Self Test Error 1");
      break;
    case 28:
      return F("Self Test Error 2");
      break;
    case 29:
      return F("Self Test Error 3");
      break;
    case 30:
      return F("Self Test Error 4");
      break;
    case 31:
      return F("DC inj error");
      break;
    case 32:
      return F("Grid OV");
      break;
    case 33:
      return F("Grid UV");
      break;
    case 34:
      return F("Grid OF");
      break;
    case 35:
      return F("Grid UF");
      break;
    case 36:
      return F("Z grid Hi");
      break;
    case 37:
      return F("Internal error");
      break;
    case 38:
      return F("Riso Low");
      break;
    case 39:
      return F("Vref Error");
      break;
    case 40:
      return F("Error Meas V");
      break;
    case 41:
      return F("Error Meas F");
      break;
    case 42:
      return F("Error Meas Z");
      break;
    case 43:
      return F("Error Meas Ileak");
      break;
    case 44:
      return F("Error Read V");
      break;
    case 45:
      return F("Error Read I");
      break;
    case 46:
      return F("Table fail");
      break;
    case 47:
      return F("Fan Fail");
      break;
    case 48:
      return F("UTH");
      break;
    case 49:
      return F("Interlock fail");
      break;
    case 50:
      return F("Remote Off");
      break;
    case 51:
      return F("Vout Avg errror");
      break;
    case 52:
      return F("Battery low");
      break;
    case 53:
      return F("Clk fail");
      break;
    case 54:
      return F("Input UC");
      break;
    case 55:
      return F("Zero Power");
      break;
    case 56:
      return F("Fan Stucked");
      break;
    case 57:
      return F("DC Switch Open");
      break;
    case 58:
      return F("Tras Switch Open");
      break;
    case 59:
      return F("AC Switch Open");
      break;
    case 60:
      return F("Bulk UV");
      break;
    case 61:
      return F("Autoexclusion");
      break;
    case 62:
      return F("Grid df / dt");
      break;
    case 63:
      return F("Den switch Open");
      break;
    case 64:
      return F("Jbox fail");
      break;
    default:
      return F("Sconosciuto");
      break;
    }
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    byte InverterState;
    byte Channel1State;
    byte Channel2State;
    byte AlarmState;
    bool ReadState;
  } DataState;

  DataState State;

  bool ReadState() {
    State.ReadState = Send(Address, (byte)50, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (State.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
      ReceiveData[2] = 255;
      ReceiveData[3] = 255;
      ReceiveData[4] = 255;
      ReceiveData[5] = 255;
    }

    State.TransmissionState = ReceiveData[0];
    State.GlobalState = ReceiveData[1];
    State.InverterState = ReceiveData[2];
    State.Channel1State = ReceiveData[3];
    State.Channel2State = ReceiveData[4];
    State.AlarmState = ReceiveData[5];

    return State.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    String Par1;
    String Par2;
    String Par3;
    String Par4;
    bool ReadState;
  } DataVersion;

  DataVersion Version;

  bool ReadVersion() {
    Version.ReadState = Send(Address, (byte)58, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (Version.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    Version.TransmissionState = ReceiveData[0];
    Version.GlobalState = ReceiveData[1];

    switch ((char)ReceiveData[2])
    {
    case 'i':
      Version.Par1 = F("Aurora 2 kW indoor");
      break;
    case 'o':
      Version.Par1 = F("Aurora 2 kW outdoor");
      break;
    case 'I':
      Version.Par1 = F("Aurora 3.6 kW indoor");
      break;
    case 'O':
      Version.Par1 = F("Aurora 3.0 - 3.6 kW outdoor");
      break;
    case '5':
      Version.Par1 = F("Aurora 5.0 kW outdoor");
      break;
    case '6':
      Version.Par1 = F("Aurora 6 kW outdoor");
      break;
    case 'P':
      Version.Par1 = F("3 - phase interface (3G74)");
      break;
    case 'C':
      Version.Par1 = F("Aurora 50kW module");
      break;
    case '4':
      Version.Par1 = F("Aurora 4.2kW new");
      break;
    case '3':
      Version.Par1 = F("Aurora 3.6kW new");
      break;
    case '2':
      Version.Par1 = F("Aurora 3.3kW new");
      break;
    case '1':
      Version.Par1 = F("Aurora 3.0kW new");
      break;
    case 'D':
      Version.Par1 = F("Aurora 12.0kW");
      break;
    case 'X':
      Version.Par1 = F("Aurora 10.0kW");
      break;
    default:
      Version.Par1 = F("Sconosciuto");
      break;
    }

    switch ((char)ReceiveData[3])
    {
    case 'A':
      Version.Par2 = F("UL1741");
      break;
    case 'E':
      Version.Par2 = F("VDE0126");
      break;
    case 'S':
      Version.Par2 = F("DR 1663 / 2000");
      break;
    case 'I':
      Version.Par2 = F("ENEL DK 5950");
      break;
    case 'U':
      Version.Par2 = F("UK G83");
      break;
    case 'K':
      Version.Par2 = F("AS 4777");
      break;
    default:
      Version.Par2 = F("Sconosciuto");
      break;
    }

    switch ((char)ReceiveData[4])
    {
    case 'N':
      Version.Par3 = F("Transformerless Version");
      break;
    case 'T':
      Version.Par3 = F("Transformer Version");
      break;
    default:
      Version.Par3 = F("Sconosciuto");
      break;
    }

    switch ((char)ReceiveData[5])
    {
    case 'W':
      Version.Par4 = F("Wind version");
      break;
    case 'N':
      Version.Par4 = F("PV version");
      break;
    default:
      Version.Par4 = F("Sconosciuto");
      break;
    }

    return Version.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    float Value;
    bool ReadState;
  } DataDSP;

  DataDSP DSP;

  bool ReadDSP(byte type, byte global) {
    if ((((int)type >= 1 && (int)type <= 9) || ((int)type >= 21 && (int)type <= 63)) && ((int)global >= 0 && (int)global <= 1)) {
      DSP.ReadState = Send(Address, (byte)59, type, global, (byte)0, (byte)0, (byte)0, (byte)0);

      if (DSP.ReadState == false) {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
      }

    }
    else {
      DSP.ReadState = false;
      clearReceiveData();
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    DSP.TransmissionState = ReceiveData[0];
    DSP.GlobalState = ReceiveData[1];

    foo.asBytes[0] = ReceiveData[5];
    foo.asBytes[1] = ReceiveData[4];
    foo.asBytes[2] = ReceiveData[3];
    foo.asBytes[3] = ReceiveData[2];

    DSP.Value = foo.asFloat;

    return DSP.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    unsigned long Secondi;
    bool ReadState;
  } DataTimeDate;

  DataTimeDate TimeDate;

  bool ReadTimeDate() {
    TimeDate.ReadState = Send(Address, (byte)70, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (TimeDate.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    TimeDate.TransmissionState = ReceiveData[0];
    TimeDate.GlobalState = ReceiveData[1];
    TimeDate.Secondi = ((unsigned long)ReceiveData[2] << 24) + ((unsigned long)ReceiveData[3] << 16) + ((unsigned long)ReceiveData[4] << 8) + (unsigned long)ReceiveData[5];
    return TimeDate.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    byte Alarms1;
    byte Alarms2;
    byte Alarms3;
    byte Alarms4;
    bool ReadState;
  } DataLastFourAlarms;

  DataLastFourAlarms LastFourAlarms;

  bool ReadLastFourAlarms() {
    LastFourAlarms.ReadState = Send(Address, (byte)86, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (LastFourAlarms.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
      ReceiveData[2] = 255;
      ReceiveData[3] = 255;
      ReceiveData[4] = 255;
      ReceiveData[5] = 255;
    }

    LastFourAlarms.TransmissionState = ReceiveData[0];
    LastFourAlarms.GlobalState = ReceiveData[1];
    LastFourAlarms.Alarms1 = ReceiveData[2];
    LastFourAlarms.Alarms2 = ReceiveData[3];
    LastFourAlarms.Alarms3 = ReceiveData[4];
    LastFourAlarms.Alarms4 = ReceiveData[5];

    return LastFourAlarms.ReadState;
  }

  bool ReadJunctionBoxState(byte nj) {
    return Send(Address, (byte)200, nj, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadJunctionBoxVal(byte nj, byte par) {
    return Send(Address, (byte)201, nj, par, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  // Inverters
  typedef struct {
    String PN;
    bool ReadState;
  } DataSystemPN;

  DataSystemPN SystemPN;

  bool ReadSystemPN() {
    SystemPN.ReadState = Send(Address, (byte)52, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    SystemPN.PN = String(String((char)ReceiveData[0]) + String((char)ReceiveData[1]) + String((char)ReceiveData[2]) + String((char)ReceiveData[3]) + String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return SystemPN.ReadState;
  }

  typedef struct {
    String SerialNumber;
    bool ReadState;
  } DataSystemSerialNumber;

  DataSystemSerialNumber SystemSerialNumber;

  bool ReadSystemSerialNumber() {
    SystemSerialNumber.ReadState = Send(Address, (byte)63, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    SystemSerialNumber.SerialNumber = String(String((char)ReceiveData[0]) + String((char)ReceiveData[1]) + String((char)ReceiveData[2]) + String((char)ReceiveData[3]) + String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return SystemSerialNumber.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    String Week;
    String Year;
    bool ReadState;
  } DataManufacturingWeekYear;

  DataManufacturingWeekYear ManufacturingWeekYear;

  bool ReadManufacturingWeekYear() {
    ManufacturingWeekYear.ReadState = Send(Address, (byte)65, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (ManufacturingWeekYear.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    ManufacturingWeekYear.TransmissionState = ReceiveData[0];
    ManufacturingWeekYear.GlobalState = ReceiveData[1];
    ManufacturingWeekYear.Week = String(String((char)ReceiveData[2]) + String((char)ReceiveData[3]));
    ManufacturingWeekYear.Year = String(String((char)ReceiveData[4]) + String((char)ReceiveData[5]));

    return ManufacturingWeekYear.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    String Release;
    bool ReadState;
  } DataFirmwareRelease;

  DataFirmwareRelease FirmwareRelease;

  bool ReadFirmwareRelease() {
    FirmwareRelease.ReadState = Send(Address, (byte)72, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

    if (FirmwareRelease.ReadState == false) {
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    FirmwareRelease.TransmissionState = ReceiveData[0];
    FirmwareRelease.GlobalState = ReceiveData[1];
    FirmwareRelease.Release = String(String((char)ReceiveData[2]) + "." + String((char)ReceiveData[3]) + "." + String((char)ReceiveData[4]) + "." + String((char)ReceiveData[5]));

    return FirmwareRelease.ReadState;
  }

  typedef struct {
    byte TransmissionState;
    byte GlobalState;
    unsigned long Energy;
    bool ReadState;
  } DataCumulatedEnergy;

  DataCumulatedEnergy CumulatedEnergy;

  bool ReadCumulatedEnergy(byte par) {
    if ((int)par >= 0 && (int)par <= 6) {
      CumulatedEnergy.ReadState = Send(Address, (byte)78, par, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);

      if (CumulatedEnergy.ReadState == false) {
        ReceiveData[0] = 255;
        ReceiveData[1] = 255;
      }

    }
    else {
      CumulatedEnergy.ReadState = false;
      clearReceiveData();
      ReceiveData[0] = 255;
      ReceiveData[1] = 255;
    }

    CumulatedEnergy.TransmissionState = ReceiveData[0];
    CumulatedEnergy.GlobalState = ReceiveData[1];
    if (CumulatedEnergy.ReadState == true) {
            ulo.asBytes[0] = ReceiveData[5];
           ulo.asBytes[1] = ReceiveData[4];
            ulo.asBytes[2] = ReceiveData[3];
            ulo.asBytes[3] = ReceiveData[2];

           CumulatedEnergy.Energy = ulo.asUlong;
    }
    return CumulatedEnergy.ReadState;
  }

  bool WriteBaudRateSetting(byte baudcode) {
    if ((int)baudcode >= 0 && (int)baudcode <= 3) {
      return Send(Address, (byte)85, baudcode, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    }
    else {
      clearReceiveData();
      return false;
    }
  }

  // Central
  bool ReadFlagsSwitchCentral() {
    return Send(Address, (byte)67, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadCumulatedEnergyCentral(byte var, byte ndays_h, byte ndays_l, byte global) {
    return Send(Address, (byte)68, var, ndays_h, ndays_l, global, (byte)0, (byte)0);
  }

  bool ReadFirmwareReleaseCentral(byte var) {
    return Send(Address, (byte)72, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadBaudRateSettingCentral(byte baudcode, byte serialline) {
    return Send(Address, (byte)85, baudcode, serialline, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadSystemInfoCentral(byte var) {
    return Send(Address, (byte)101, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadJunctionBoxMonitoringCentral(byte cf, byte rn, byte njt, byte jal, byte jah) {
    return Send(Address, (byte)103, cf, rn, njt, jal, jah, (byte)0);
  }

  bool ReadSystemPNCentral() {
    return Send(Address, (byte)105, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

  bool ReadSystemSerialNumberCentral() {
    return Send(Address, (byte)107, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
  }

};


//#############################################################################
// Global variables

// WiFi
ESP8266 wifi(Serial2, SER_ESP8266_BAUD_RATE);
#ifndef DBG_ESP8266
bool wifi_link_up = false;
#endif

// PV inverter
cls_PV_inverter PV_inverter = cls_PV_inverter(PV_INVERTER_ADDRESS);

// Threads
#ifndef DISABLE_HEARTBEAT
Thread th_heart_beat = Thread();
#endif
#ifndef DBG_ESP8266
Thread th_wifi_AP_connection = Thread();
#endif
#ifndef DISABLE_DATA_SAMPLER
Thread th_data_sampler = Thread();
#endif

ThreadController th_controller = ThreadController();
void th_heart_beat_callback(void);
void th_wifi_AP_connection_callback(void);
void th_data_sampler_callback(void);
void th_wdt_callback(void);

void wdt_rearm(void){
#ifndef DISABLE_WDT
 Serial.println("[WDT] Rearming ...");
  wdt_reset(); 
#endif
}

//#############################################################################
// Thread callbacks

#ifndef DISABLE_HEARTBEAT
void th_heart_beat_callback(void){
  static int first_run = 1;
  static int led_status = 0;

  wdt_rearm();

  if (first_run == 1){
    digitalWrite(LED_BUILTIN, LOW);
    led_status = 0;
    first_run = 0;
  } else {
    if (led_status == 0){
      digitalWrite(LED_BUILTIN, HIGH);
      led_status = 1;
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      led_status = 0;     
    }
  }
}
#endif

#ifndef DBG_ESP8266
void th_wifi_AP_connection_callback(void){
  static boolean first_run = true;
  boolean err_code = false;
  static int status_check_failures = 0;

  wdt_rearm();

  if (first_run){
    Serial.println("[ESP8266] th_wifi_AP_connection_callback first run");
    // Initialize UART channel
    Serial2.begin(SER_ESP8266_BAUD_RATE);
    first_run = false;
  }
  
  if (!wifi_link_up){
    wdt_rearm();
    Serial.println("[ESP8266] Resetting module ...");
    err_code = wifi.restart();
    if (!err_code)
      goto reset_fail;

    wdt_rearm();
    Serial.print("[ESP8266] FW Version: ");
    Serial.println(wifi.getVersion().c_str());    

    wdt_rearm();
    err_code = wifi.setOprToStation();
    if (err_code) {
        Serial.println("[ESP8266] Setting station mode ok");
    } else {
        goto setting_station_mode_fail;
    }

    wdt_rearm();
    Serial.println("[ESP8266] AP list");
    Serial.println(wifi.getAPList());

    wdt_rearm();
    if (wifi.joinAP(WIFI_SSID, WIFI_PASSWD)) {
        Serial.println("[ESP8266] Connection to AP established");
        Serial.print("IP: ");       
        Serial.println(wifi.getLocalIP().c_str());

        if (wifi.disableMUX()) {
                Serial.print("[ESP8266] Single connection mode selected\r\n");
            } else {
                goto single_connection_mode_fail;
            }
        wifi_link_up = true;
        goto function_exit;
    } else {
      goto AP_connection_fail;
    }   
    
  } else { // verify that WiFi module is up and running and that link is up
    //err_code = wifi.kick();
    wdt_rearm();
    String ret_string = wifi.getIPStatus();
    Serial.print("[ESP8266] Checking connection status ... Status is: ");
    Serial.println(ret_string);
    if (ret_string.length() > 0){
      String status_code_s = ret_string.substring(ret_string.length()-1);
      int status_code_int = status_code_s.toInt();
      switch (status_code_int){
        case 2:
        case 3:
        case 4:
          // everything is ok
          Serial.print("[ESP8266] Status code is: ");
          Serial.println(status_code_int);
          goto function_exit;
          break;
        default:
          // it seems we have a problem ...
          status_check_failures++;
          goto status_check_fail;
          break;
      }
    } else {
      status_check_failures++;
      goto status_check_fail;
    }
  }
  
reset_fail:
  Serial.println("[ESP8266] Reset command failed");
  goto function_exit;
setting_station_mode_fail:
  Serial.println("[ESP8266] Setting station mode failed");
  goto function_exit;
AP_connection_fail:  
  Serial.println("[ESP8266] AP connection failed");
  goto function_exit;
status_check_fail:
  Serial.println("[ESP8266] Status check failed");
  if (status_check_failures == 3){
    wifi_link_up = false;
    status_check_failures = 0;
  }    
  goto function_exit;
single_connection_mode_fail:
  Serial.print("[ESP8266] Single connection mode setup failed\r\n");
  goto function_exit;
function_exit:;
}
#endif

#ifndef DISABLE_DATA_SAMPLER
boolean upload_string(String s){
#define BUF_LEN   1024
  char buf[BUF_LEN];

  wdt_rearm();
  delay(1000);

  Serial.print("[cloud] Trying to upload this string = ");
  Serial.println(s);

  s += "\r\n";
  if (s.length() <= BUF_LEN){
    s.toCharArray(buf, BUF_LEN);
  } else {
    Serial.println("[cloud] String too long. Can't upload data to the cloud.");
    return false;
  }

  if (wifi.send((const uint8_t*)buf, strlen(buf))){
    Serial.println("[cloud] Data uploaded to the cloud");
    return true;
  } else {
    Serial.println("[cloud] Failed to upload data to the cloud");
    return false;
  }
  return false;
}

boolean upload_variable(const String token, const String var_id, const String var_value){
  String tmp_string = "";
  String var_string = "";
  int var_string_len = 0;
  boolean err_code = false;

  wdt_rearm();

  Serial.println("[cloud] Uploading data to the cloud ...");
  if (wifi_link_up){
    if (wifi.createTCP(SERVER_NAME, SERVER_PORT)) {
      Serial.println("[cloud] TCP connection established");
    } else {
      Serial.println("[cloud] Can't establish TCP connection");
      goto function_exit_fail;
    }
  } else {
    Serial.println("[cloud] WiFi link is down. Can't upload data to the cloud.");
    goto function_exit_fail;
  }
  
  tmp_string = "POST /api/v1.6/variables/";
  tmp_string += var_id;
  tmp_string += "/values HTTP/1.1";
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  tmp_string = "X-Auth-Token: " + token;
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  tmp_string = "Host: " + String(SERVER_NAME);
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  tmp_string = "Connection: close";
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  tmp_string = "Content-Type: application/json";
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  var_string = "{\"value\": ";
  var_string += String(var_value);
  var_string += "}";

  tmp_string = "Content-Length: ";
  var_string_len = var_string.length();
  tmp_string += String(var_string_len);
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  } 
  tmp_string = "";
  if (!upload_string(tmp_string)){
    err_code = false;
    goto close_TCP_connection;
  }
  if (!upload_string(var_string)){
    err_code = false;
    goto close_TCP_connection;
  }

close_TCP_connection:
  if (wifi.releaseTCP()) {
    Serial.println("[cloud] TCP connection closed");
    return err_code;
  } else {
    Serial.println("[cloud] Failed to close TCP connection");
    return false;
  }
function_exit_fail:
  return false;
}

void upload_PV_inverter_data(
  float grid_power,
  unsigned long total_cumulated_energy){

  if (upload_variable(String(UBIDOTS_TOKEN), UBIDOTS_GRID_POWER_VAR_ID, String(grid_power))){
     Serial.println("[cloud] Grid power value uploaded");
  } else {
    Serial.println("[cloud] Failed to upload grid power value");
  }
  
  if (upload_variable(String(UBIDOTS_TOKEN), UBIDOTS_TOTAL_CUMULATED_ENERGY_VAR_ID, String(total_cumulated_energy))){
     Serial.println("[cloud] Total cumulated energy value uploaded");
  } else {
    Serial.println("[cloud] Failed to upload total cumulated energy value");
  }
}

void upload_temp_hum(float tempValue, float humValue){
#ifndef DISABLE_DATA_SAMPLER_TEMP_HUM
  if (upload_variable(String(UBIDOTS_TOKEN), UBIDOTS_TEMPERATURE_VAR_ID, String(tempValue))){
     Serial.println("[cloud] Temperature value uploaded");
  } else {
    Serial.println("[cloud] Failed to upload temperature value");
  }

  if (!upload_variable(String(UBIDOTS_TOKEN), UBIDOTS_HUMIDITY_VAR_ID, String(humValue))){
    Serial.println("[cloud] Humidity value uploaded");
  } else {
    Serial.println("[cloud] Failed to upload humidity value");
  }
#endif // #ifndef DISABLE_DATA_SAMPLER_TEMP_HUM
}

void th_data_sampler_callback(void){
  int err_code;
  dht DHT;

  wdt_rearm();

  // Get temperature and humidity
#if (DHT_TYPE == DHT11)
  err_code = DHT.read11(DHT_PIN);
#elif (DHT_TYPE == DHT22)
  err_code = DHT.read22(DHT_PIN);
#endif
  switch (err_code)
  {
    case DHTLIB_OK:  
      Serial.println("[DHT] Read operation successful");
      Serial.print("[DHT] Temperature = ");
      Serial.println(DHT.temperature, 1);
      Serial.print("[DHT] Humidity = ");
      Serial.println(DHT.humidity, 1);
      
      break;
    case DHTLIB_ERROR_CHECKSUM: 
      Serial.println("[DHT] Read operation failed (checksum error)"); 
      break;
    case DHTLIB_ERROR_TIMEOUT: 
      Serial.println("[DHT] Read operation failed (time out error)"); 
      break;
    default: 
      Serial.println("[DHT] Read operation failed (unknown error)"); 
      break;
  }

  wdt_rearm();

#ifndef DISABLE_DATA_SAMPLER_PV_INVERTER
  // Get data from PV inverter
  if (!PV_inverter.ReadCumulatedEnergy(5)){
    Serial.println("[PV] Couldn't read inverter's total cumulated energy");
    goto function_exit_fail;
  }
  Serial.println("[PV] Inverter's total cumulated energy = " + String(PV_inverter.CumulatedEnergy.Energy/1000) + " kWh");
  wdt_rearm();
  if (!PV_inverter.ReadDSP(3, 1)){
    Serial.println("[PV] Couldn't read grid power");
    goto function_exit_fail;
  }
  Serial.println("[PV] Grid power = " + String(PV_inverter.DSP.Value) + " W");
#endif // #ifndef DISABLE_DATA_SAMPLER_PV_INVERTER

  // We got all the data; upload all this stuff to the cloud
  upload_temp_hum(DHT.temperature, DHT.humidity);

#ifndef DISABLE_DATA_SAMPLER_PV_INVERTER
  upload_PV_inverter_data(
    PV_inverter.DSP.Value,
    PV_inverter.CumulatedEnergy.Energy/1000
    );
#endif

  return;

function_exit_fail:
  Serial.println("[DS] Aborting data sampling ...");
}
#endif // #ifndef DISABLE_DATA_SAMPLER


void setup() {
  // put your setup code here, to run once:
  boolean error_code = false;

#ifndef DISABLE_WDT
  // Enable WDT as soon as possible
  wdt_enable(WDTO_8S);
  //wdt_enable(WDTO_1S);
#endif

  // Init physical interfaces
  Serial.begin(SER_DEBUG_BAUD_RATE);
  Serial.println("\r\n##################################################");
  Serial.println("PV logger - " + String(SW_VERSION) + "\r\n");
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("[DHT] DHTLib version = ");
  Serial.println(DHT_LIB_VERSION);
  Serial.println("[DHT] Sensor version = DHT" + String(DHT_TYPE));

  // Initialization of RS485 interface is perfomed by constructor of cls_PV_inverter
#ifndef DISABLE_PV_INVERTER_PROBING
  if (!PV_inverter.ReadSystemPN()){
    Serial.println("[PV] Couldn't read inverter's p/n. Resetting ...");
    while(1); // Just wait for WDT to expire ...
  }
  Serial.println("[PV] Inverter's p/n = " + PV_inverter.SystemPN.PN);
  
  if (!PV_inverter.ReadSystemSerialNumber()){
    Serial.println("[PV] Couldn't read inverter's s/n. Resetting ...");
    while(1); // Just wait for WDT to expire ...
  }
  Serial.println("[PV] Inverter's s/n = " + PV_inverter.SystemSerialNumber.SerialNumber);

  if (!PV_inverter.ReadVersion()){
    Serial.println("[PV] Couldn't read inverter's version. Resetting ...");
    while(1); // Just wait for WDT to expire ...
  }
  Serial.println("[PV] Inverter's version = " + PV_inverter.Version.Par1 + "." + PV_inverter.Version.Par2 + "." + PV_inverter.Version.Par3 + "." + PV_inverter.Version.Par4);
  
  if (!PV_inverter.ReadManufacturingWeekYear()){
    Serial.println("[PV] Couldn't read inverter's manufacturing week/year. Resetting ...");
    while(1); // Just wait for WDT to expire ...
  }
  Serial.println("[PV] Inverter's manufacturing week/year = " + PV_inverter.ManufacturingWeekYear.Week + "/" + PV_inverter.ManufacturingWeekYear.Year);
  
  if (!PV_inverter.ReadFirmwareRelease()){
    Serial.println("[PV] Couldn't read inverter's firmware release. Resetting ...");
    while(1); // Just wait for WDT to expire ...
  }
  Serial.println("[PV] Inverter's firmware release = " + PV_inverter.FirmwareRelease.Release);
#endif

  
#ifdef DGB_SERIAL1
  pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
  digitalWrite(RS485_TX_ENABLE_PIN, RS485_RX_MODE);
  Serial1.begin(RS485_SERIAL_BAUD_RATE);
#endif

#ifdef DBG_ESP8266
  Serial2.begin(SER_ESP8266_BAUD_RATE);

  Serial.print("FW Version: ");
  Serial.println(wifi.getVersion().c_str());
  
  
  if (wifi.setOprToStation()) {
      Serial.print("to station ok\r\n");
  } else {
      Serial.print("to station err\r\n");
  }

  if (wifi.joinAP(WIFI_SSID, WIFI_PASSWD)) {
      Serial.print("Join AP success\r\n");
      Serial.print("IP: ");       
      Serial.println(wifi.getLocalIP().c_str());
  } else {
      Serial.print("Join AP failure\r\n");
  }
      
#endif

  // Initialize threads
#ifndef DISABLE_HEARTBEAT
  th_heart_beat.enabled = true;
  th_heart_beat.setInterval(HEART_BEAT_INTERVAL);
  th_heart_beat.ThreadName = "th_heart_beat";
  Serial.print("[sys] th_heart_beat ID = ");
  Serial.println(th_heart_beat.ThreadID);
  th_heart_beat.onRun(th_heart_beat_callback);
#endif

#ifndef DBG_ESP8266
  th_wifi_AP_connection.enabled = true;
  th_wifi_AP_connection.setInterval(WIFI_AP_CONNECTION_TEST_INTERVAL);
  th_wifi_AP_connection.ThreadName = "th_wifi_AP_connection";
  Serial.print("[sys] th_wifi_AP_connection ID = ");
  Serial.println(th_wifi_AP_connection.ThreadID);
  th_wifi_AP_connection.onRun(th_wifi_AP_connection_callback);
#endif

#ifndef DISABLE_DATA_SAMPLER
  th_data_sampler.enabled = true;
  th_data_sampler.setInterval(DATA_SAMPLER_INTERVAL);
  th_data_sampler.ThreadName = "th_data_sampler";
  Serial.print("[sys] th_data_sampler ID = ");
  Serial.println(th_data_sampler.ThreadID);
  th_data_sampler.onRun(th_data_sampler_callback);
#endif

  // Init ThreadController
#ifndef DISABLE_HEARTBEAT
  error_code = th_controller.add(&th_heart_beat);
  if (!error_code)
    Serial.println("[sys] Can't add th_heart_beat thread");
  else
    Serial.println("[sys] th_heart_beat thread added");
#endif

#ifndef DBG_ESP8266
  error_code = th_controller.add(&th_wifi_AP_connection);
  if (!error_code)
    Serial.println("[sys] Can't add th_wifi_AP_connection thread");
  else
    Serial.println("[sys] th_wifi_AP_connection thread added");
#endif
#ifndef DISABLE_DATA_SAMPLER
  error_code = th_controller.add(&th_data_sampler);
  if (!error_code)
    Serial.println("[sys] Can't add th_data_sampler thread");
  else
    Serial.println("[sys] th_data_sampler thread added");
#endif

  Serial.print("[sys] # of threads = ");
  Serial.println(th_controller.size(false));
  Serial.print("[sys] # of threads (caFched value) = ");
  Serial.println(th_controller.size(true));

  wdt_rearm();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Start threads
  th_controller.run();

#ifdef DGB_SERIAL1
  digitalWrite(RS485_TX_ENABLE_PIN, RS485_TX_MODE);
  //delay(1000);
  Serial1.println("UUU");
  //digitalWrite(RS485_TX_ENABLE_PIN, RS485_RX_MODE);
#endif

#ifdef DBG_ESP8266
#if 1
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  while (Serial.available()) {
    Serial2.write(Serial.read());
  }
#else


/*
  Serial2.println("AT+RST");
  //Serial2.println("AT+GMR");
  Serial.print("[ESP8266] RX chars = ");
  Serial.println(Serial2.available());
  while(Serial2.available()>0)
    Serial2.read();
*/
#endif
#endif

  // Do nothing else: everything is handled by threads
}
