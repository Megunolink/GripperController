#include <Servo.h>
#include <CommandHandler.h>
#include <EEPROMStore.h>
#include <MegunoLink.h>
#include <ArduinoTimer.h>

const int ServoPin = 7;
Servo Gripper;

CommandHandler<> SerialCmds;

struct GripperConfiguration
{
  int ClosedPosition; 
  int OpenPosition; 

  void Reset()
  {
    ClosedPosition = 90; 
    OpenPosition = 90; 
  }
};

EEPROMStore<GripperConfiguration> PersistentConfig;

void Cmd_SetRawPosition(CommandParameter& p)
{
  int GripperPosition = p.NextParameterAsInteger();
  
  if (GripperPosition < 0)
  {
    GripperPosition = 0; 
  }
  if (GripperPosition > 180)
  {
    GripperPosition = 180;
  }
  SetJawPosition(GripperPosition);
}

void Cmd_Open(CommandParameter& p)
{ 
  SetJawPosition(PersistentConfig.Data.OpenPosition);
  InterfacePanel Panel;
  Panel.SetNumber(F("numJaws"), 100);
}

void Cmd_Close(CommandParameter& p)
{ 
  SetJawPosition(PersistentConfig.Data.ClosedPosition);
  InterfacePanel Panel;
  Panel.SetNumber(F("numJaws"), 0);
}

void Cmd_SaveToEeprom(CommandParameter& p)
{
  PersistentConfig.Save();
  Serial.println(F("Saved configuration to EEPROM"));
}

void Cmd_SetGripperPosition(CommandParameter& p)
{
  int ProportionOpen = p.NextParameterAsInteger(100);
  if (ProportionOpen < 0)
  {
    ProportionOpen = 0;
  }
  if (ProportionOpen > 100)
  {
    ProportionOpen = 100; 
  }
  
  int OpenPosition = PersistentConfig.Data.OpenPosition;
  int ClosedPosition = PersistentConfig.Data.ClosedPosition;
  int NewPosition = (ProportionOpen * 
    (OpenPosition - ClosedPosition))/100 + ClosedPosition;

  SetJawPosition(NewPosition);
}

void Cmd_GetConfiguration(CommandParameter& p)
{
  InterfacePanel MyPanel;

  // Set control value
  MyPanel.SetNumber(F("numClosedPosition"), 
    PersistentConfig.Data.ClosedPosition);
  MyPanel.SetNumber(F("numOpenPosition"), 
    PersistentConfig.Data.OpenPosition);
}

void SetJawPosition(int NewPosition)
{
  Gripper.write(NewPosition);
  
  InterfacePanel MyPanel;
  MyPanel.SetNumber(F("numRawPosition"), NewPosition);
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Robot Gripper Controller"));

  SerialCmds.AddCommand(F("SetRawPosition"), Cmd_SetRawPosition);
  SerialCmds.AddVariable(F("ClosedPosition"), PersistentConfig.Data.ClosedPosition);
  SerialCmds.AddVariable(F("OpenPosition"), PersistentConfig.Data.OpenPosition);
  SerialCmds.AddCommand(F("Open"), Cmd_Open);
  SerialCmds.AddCommand(F("Close"), Cmd_Close);
  SerialCmds.AddCommand(F("Save"), Cmd_SaveToEeprom);
  SerialCmds.AddCommand(F("SetGripperPosition"), Cmd_SetGripperPosition);
  SerialCmds.AddCommand(F("GetConfiguration"), 
    Cmd_GetConfiguration);

  Gripper.attach(ServoPin);
}

void loop() 
{
  SerialCmds.Process();

  static ArduinoTimer ForceMeasurementTimer;
  if (ForceMeasurementTimer.TimePassed_Milliseconds(100))
  {
    int RawForce = analogRead(0);
    
    TimePlot ForcePlot;
    ForcePlot.SendData(F("Raw force [adc counts]"), RawForce);
  }

}
