
/**************************************************************************/
/*!
This is a demo for the Adafruit MCP9808 breakout
----> http://www.adafruit.com/products/1782
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
*/
/**************************************************************************/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <Adafruit_MCP9808.h>

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main() {
   AdafruitMCP9808 *AdafruitMCP9808 = new AdafruitMCP9808();
   
  int err =  AdafruitMCP9808->openAdafruitMCP9808();
    if (err < 0){
        printf("Error: %d",  AdafruitMCP9808->error);
    } else {

        int hardwareVersion = AdafruitMCP9808->getHardwareVersion() ;
        int softwareVersion = AdafruitMCP9808->getSoftwareVersion() ;
        printf("Hardware Version: %d\n",hardwareVersion) ;
        printf("Software Version: %d\n",softwareVersion) ;



 // tempsensor->wake();   // wake up, ready to read!

  // Read and print out the temperature, then convert to *F
  float c = AdafruitMCP9808 -> getTemperature();
  float f = c * 9.0 / 5.0 + 32;
  
  printf("Temp: %f\n",c) ;
 // printf("Temp: "); Serial.print(c); Serial.print("*C\t"); 
  //Serial.print(f); Serial.println("*F");
  
  //Serial.println("Shutdown MCP9808.... ");
  //tempsensor.shutdown(); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere
  
  delay(1000);
        return 0;
        AdafruitMCP9808->closeAdafruitMCP9808();
}
