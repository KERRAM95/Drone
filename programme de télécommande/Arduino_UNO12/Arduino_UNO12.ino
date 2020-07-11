#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <PS2X_lib.h>  //for v1.6
PS2X ps2x; 
int error = 0; 
byte type = 0;
byte vibrate = 0;


#define joystickPin1 0  //analog 0
#define joystickPin2 1  //analog 0
#define joystickPin3 2  //analog 0
#define joystickPin4 3  //analog 0


 float numero5=2.0;

void setup(){
  Serial.begin(9600); // pour dÃ©bogage seulement.

//CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
 error = ps2x.config_gamepad(2,4,3,5, true, true);   //setup pins and settings:  GamePad(clock=bleu , command=orange, attention=jaune, data=b        run, Pressures?, Rumble?) check for error
 //The numbers he reported success with is as follow:

//5V = 180 ohm, voltage drop of 1.85V (3.15v to the controller)
//ATT = 5.6K ohm, voltage drop of 1.08V (3.92 to the controller)
//CLK = 5.6K ohm, voltage drop of 1.15V (3.85 to the controller)
//CMD = 860K ohm, voltage drop of 1.39V (3.61 to the controller)
//so you can give that a try as well. Though really, ignore the resistor on 5V and just connect the red power wire to 3.3V.
 if(error == 0){
   Serial.println("Found Controller, configured successful");
   Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
  Serial.println("holding L1 or R1 will print out the analog stick values.");
  Serial.println("Go to www.billporter.info for updates and to report bugs.");
 }
   
  else if(error == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
   //Serial.print(ps2x.Analog(1), HEX);
   
   type = ps2x.readType(); 
     switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
       case 2:
         Serial.println("GuitarHero Controller Found");
       break;
     }




  Mirf.cePin = 9; // CE sur D9
  Mirf.csnPin = 10; // CSN sur D10
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init(); 

  Mirf.channel = 0; // On va utiliser le canal 0 pour communiquer (128 canaux disponible, de 0 Ã  127)
  Mirf.payload = 10; //  ici il faut dÃ©clarer la taille du "payload" soit du message qu'on va transmettre, au max 32 octets
  Mirf.config(); 

  Mirf.setTADDR((byte *)"nrf02"); // Le 1er module va envoyer ses info au 2eme module
  Mirf.setRADDR((byte *)"nrf01"); // On dÃ©finit ici l'adresse du 1er module

  Serial.println("Le client est pret..."); 
}

void loop(){
  
  
  
  /* You must Read Gamepad to get new values
   Read GamePad and set vibration values
   ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
   if you don't enable the rumble, use ps2x.read_gamepad(); with no values
   
   you should call this at least once a second
   */
   
   
   
 if(error == 1) //skip loop if no controller found
  return; 
  
 
 else { //DualShock Controller
  
    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))                   //will be TRUE as long as button is pressed
         Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
         Serial.println("Select is being held");
         
         
     if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
       Serial.print("Up held this hard: ");
       Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      }
      if(ps2x.Button(PSB_PAD_RIGHT)){
       Serial.print("Right held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      }
      if(ps2x.Button(PSB_PAD_LEFT)){
       Serial.print("LEFT held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      }
      if(ps2x.Button(PSB_PAD_DOWN)){
       Serial.print("DOWN held this hard: ");
     Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      }   
  
    
      vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on 
                                              //how hard you press the blue (X) button    
    
    if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
    {
                   
        if(ps2x.Button(PSB_L3))
        // Serial.println("L3 pressed");
        if(ps2x.Button(PSB_R3))
        // Serial.println("R3 pressed");
        if(ps2x.Button(PSB_L2))
        // Serial.println("L2 pressed");
        if(ps2x.Button(PSB_R2))
        // Serial.println("R2 pressed");
        if(ps2x.Button(PSB_GREEN))
         //Serial.println("Triangle pressed");
         if(ps2x.Button(PSB_RED))
        // Serial.println("Rond pressed");
         if(ps2x.Button(PSB_PINK))
        // Serial.println("Carre pressed");
         if(ps2x.Button(PSB_BLUE))
         Serial.println("Croix pressed");
         
    }   
         
    if(ps2x.Button(PSB_GREEN))
        // Serial.println("Triangle just pressed");
         
    if(ps2x.ButtonPressed(PSB_RED))             //will be TRUE if button was JUST pressed
        // Serial.println("Circle just pressed");
         
    if(ps2x.ButtonReleased(PSB_PINK))             //will be TRUE if button was JUST released
        // Serial.println("Square just released");     
    
    if(ps2x.NewButtonState(PSB_BLUE))            //will be TRUE if button was JUST pressed OR released
        // Serial.println("X just changed");    
    
    
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
    { 
        Serial.print("Stick Values:");
        Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
        Serial.print(",");
        Serial.print(ps2x.Analog(PSS_LX), DEC); 
        Serial.print(",");
        Serial.print(ps2x.Analog(PSS_RY), DEC); 
        Serial.print(",");
        Serial.println(ps2x.Analog(PSS_RX), DEC);

  delay(10);
         
    } 
    
    
 }
 
 
  int numero1 = ps2x.Analog(PSS_RX);
  int numero2 = ps2x.Analog(PSS_RY);
  int numero3 = ps2x.Analog(PSS_LX);
  int numero4 = ps2x.Analog(PSS_LY);
 
  boolean numero6=true;
  
  while(numero6 && ps2x.Button(PSB_PAD_UP) && numero5<224.0){
  numero5 = (numero5*1.25);
 
  Serial.println(numero5);
  numero6=false;

  }
  boolean numero7=true;
  while(numero7 && ps2x.Button(PSB_PAD_DOWN) && numero5>0.0){
  numero5 = (numero5/1.25);
  
  Serial.println(numero5);
  numero7=false;
 
  }

  int numero8 = int(numero5);
  
  // conversion dans un array de bytes
  byte data[Mirf.payload]; 

  data[0] = (byte )((numero1 >> 8) & 0xff);
  data[1] = (byte )(numero1 & 0xff);
  data[2] = (byte )((numero2 >> 8) & 0xff);
  data[3] = (byte )(numero2 & 0xff);
  data[4] = (byte )((numero3 >> 8) & 0xff);
  data[5] = (byte )(numero3 & 0xff);
  data[6] = (byte )((numero4 >> 8) & 0xff);
  data[7] = (byte )(numero4 & 0xff);
  data[8] = (byte )((numero8 >> 8) & 0xff);
  data[9] = (byte )(numero8 & 0xff);
  Mirf.send((byte *)&data); // On envoi les donnÃ©es

  while(Mirf.isSending()); 

  // pour dÃ©bogage seulement:
  /*
  Serial.print("Envoye numeros ");
  Serial.print(numero1,DEC);
  Serial.print(" , ");
  Serial.println(numero2,DEC);
*/
  delay(100);
}
