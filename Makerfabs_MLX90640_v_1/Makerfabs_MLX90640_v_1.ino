/*
 * Увеличена частота опроса сенсора
 * Увеличена частота CLOCK Шины I2C
 * Добавлено условие проверки дельты температур и расширения границ шкалы
 * Добавлена бикубическая интерполяция
 * Переход на библиотеку Lovyan_GVX
 * Добавлена функция рисования гистограммы
 * Добавлено сенсорное управление
*/

#define LGFX_MAKERFABS_TOUCHCAMERA // Makerfabs Touch with Camera

#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "SPI.h"
#include "Blur.h"
#include <EEPROM.h>            // read and write from flash memory
#include <LovyanGFX.hpp>
#include "FS.h"
#include "SD.h"
// define the number of bytes you want to access
#define EEPROM_SIZE 1
#include "Button2.h";

#include "histogram.h"

float b[180];

Histogram hist(180, b);

//I2C
#define ESP32_TSC_9488_I2C_SDA 26
#define ESP32_TSC_9488_I2C_SCL 27

//SD Card
#define SD_CS 4

//SPI
#define SPI_MOSI 13
#define SPI_MISO 12
#define SPI_SCK 14

#define SPI_OFF_SD digitalWrite(SD_CS, HIGH)

#define interpolation 1 //1 - Gauss, 2 - Bicubic

#define GREY 0x4a49 

static LGFX tft;                 // LGFXのインスタンスを作成。

#define BUTTON_A_PIN  36

Button2 button = Button2(BUTTON_A_PIN);

const byte MLX90640_address = 0x33;           //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8                            //Default shift for MLX90640 in open air

static float mlx90640To[768];
static float mlx90640ToBlur[3072];

paramsMLX90640 mlx90640;

int R_colour, G_colour, B_colour;              // RGB-Farbwert
int i, j;                                      // Zählvariable
float T_max, T_min;                            // maximale bzw. minimale gemessene Temperatur
float T_center;                                // Temperatur in der Bildschirmmitte
uint16_t tft_width  = 480;                     // ST7789_TFTWIDTH;
uint16_t tft_height = 320;                     // ST7789_TFTHEIGHT;
String filename;
int pictureNumber = 1;
int colorStep = 180;
GBlur blur;                                       //Gauss interpolation class

//Bicubic interpolation function
float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);

#include "FT6236.h"
const int i2c_touch_addr = TOUCH_I2C_ADD;
#define get_pos ft6236_pos

int pos[2] = {0, 0};

// ***************************************
// **************** SETUP ****************
// ***************************************


void IRAM_ATTR myEvent(){    
  get_pos(pos);  
  if (150 < pos[0] && pos[0] < 190){ 
    if (380 < pos[1] && pos[1] < 470){
      //Serial.print(pos[0]);
      //Serial.print(",");
      //Serial.println(pos[1]);
      pressedTouch();
        }
    }
}

void draw_button(){
    //SPI_ON_TFT;
    //tft.fillScreen(TFT_BLACK);
    tft.fillRect(380, 150, 90, 40, GREY);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(395, 165);
    tft.println("TAKE PHOTO");
    //SPI_OFF_TFT;
}

void pressed(Button2& btn) {
    EEPROM.begin(EEPROM_SIZE);
    pictureNumber = EEPROM.read(0) + 1;
    filename = "/picture" + String(pictureNumber) +".bmp";
    saveToSD_24bit();
    Serial.print(filename);
    Serial.println(" saved");
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
}

void pressedTouch() {
    EEPROM.begin(EEPROM_SIZE);
    pictureNumber = EEPROM.read(0) + 1;
    filename = "/picture" + String(pictureNumber) +".bmp";
    saveToSD_24bit();
    Serial.print(filename);
    Serial.println(" saved");
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
}

bool saveToSD_24bit(void){
  std::size_t dlen;

  bool result = false;
  File file = SD.open(filename.c_str(), "w");
  if (file)
  {
    int width  = tft.width();
    int height = tft.height();

    int rowSize = (3 * width + 3) & ~ 3;

    lgfx::bitmap_header_t bmpheader;
    bmpheader.bfType = 0x4D42;
    bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
    bmpheader.bfOffBits = sizeof(bmpheader);

    bmpheader.biSize = 40;
    bmpheader.biWidth = width;
    bmpheader.biHeight = height;
    bmpheader.biPlanes = 1;
    bmpheader.biBitCount = 24;
    bmpheader.biCompression = 0;

    file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
    std::uint8_t buffer[rowSize];
    memset(&buffer[rowSize - 4], 0, 4);
    for (int y = tft.height() - 1; y >= 0; y--)
    {
      tft.readRect(0, y, tft.width(), 1, (lgfx::rgb888_t*)buffer);
      file.write(buffer, rowSize);
    }
    file.close();
    result = true;
  }
  else
  {
    Serial.print("error:file open failure\n");
  }

  return result;
}

void setup(){
    Serial.begin(115200);        
    button.setPressedHandler(pressed);
    //pinMode(0, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(0), myEvent, FALLING);

    //timer.setInterval(50L, myTimerEvent);    
    //I2C init
    Wire1.begin(ESP32_TSC_9488_I2C_SDA, ESP32_TSC_9488_I2C_SCL);
    byte error, address;
    Wire1.beginTransmission(i2c_touch_addr);
    error = Wire1.endTransmission();
    if (error == 0){
        Serial.print("I2C device found at address 0x");
        Serial.print(i2c_touch_addr, HEX);
        Serial.println("  !");
    }
    else if (error == 4){
        Serial.print("Unknown error at address 0x");
        Serial.println(i2c_touch_addr, HEX);
    }
    
    //SPI init
    pinMode(SD_CS, OUTPUT);
    SPI_OFF_SD;
    tft.init();

    while (1) {
        if (SD.begin(SD_CS, SPI, 80000000)) {
            Serial.println("sd begin pass");
            break;
        }
        Serial.println("sd begin fail,wait 1 sec");
        delay(1000);
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    Wire.begin();
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz
    while (!Serial); //Wait for user to open terminal
   
    Serial.println("MLX90640 IR Array Example");

    if (isConnected() == false){
        Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
       }
       
    Serial.println("MLX90640 online!");

    //Get device parameters - We only have to do this once
    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0)
       Serial.println("Failed to load system parameters");
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640); 
    if (status != 0){
        Serial.println("Parameter extraction failed");
        Serial.print(" status = ");
        Serial.println(status);
       }

    //Once params are extracted, we can release eeMLX90640 array

    MLX90640_I2CWrite(0x33, 0x800D, 6401);    // writes the value 1901 (HEX) = 6401 (DEC) in the register at position 0x800D to enable reading out the temperatures!!!
    // ===============================================================================================================================================================

    //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
    MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
    //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
    //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails
    
    //Once EEPROM has been read at 400kHz we can increase to 1MHz
    Wire.setClock(1000000); //Teensy will now run I2C at 800kHz (because of clock division)
    
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    draw_button();
    drawLegend();
    tft.setCursor(80, 10);
    tft.setTextColor(TFT_WHITE, tft.color565(0, 0, 0));
    tft.print("T+ = ");    

   } 

// **********************************
// ************** LOOP **************
// **********************************

void loop(){
    //timer.run(); // Initiates SimpleTimer  
    button.loop();
    get_pos(pos);
        //Serial.print(pos[0]);
        //Serial.print(",");
        //Serial.println(pos[1]);    
      if (150 < pos[0] && pos[0] < 190){ 
        if (10 < pos[1] && pos[1] < 100){ // for scren rotate 0: 380-470 
        //Serial.print(pos[0]);
        //Serial.print(",");
        //Serial.println(pos[1]);
        pressedTouch();
        }
      }
    long startTime = millis();
    //Read both subpages
    for (byte x = 0 ; x < 2 ; x++) {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    
        if (status < 0){
            Serial.print("GetFrame Error: ");
            Serial.println(status);
           }
        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
       }
    long stopReadTime = millis();   
      
    // determine T_min and T_max and eliminate error pixels
    // ====================================================

    mlx90640To[1*32 + 21] = 0.5 * (mlx90640To[1*32 + 20] + mlx90640To[1*32 + 22]);    // eliminate the error-pixels
    mlx90640To[4*32 + 30] = 0.5 * (mlx90640To[4*32 + 29] + mlx90640To[4*32 + 31]);    // eliminate the error-pixels
    
    T_min = mlx90640To[0];
    T_max = mlx90640To[0];

    for (i = 1; i < 768; i++){
        if((mlx90640To[i] > -41) && (mlx90640To[i] < 301)){
            if(mlx90640To[i] < T_min){
                T_min = mlx90640To[i];
               }
            if(mlx90640To[i] > T_max){
                T_max = mlx90640To[i];
               }
           }
        else if(i > 0){   // temperature out of range         
            mlx90640To[i] = mlx90640To[i-1];
           }
        else{
            mlx90640To[i] = mlx90640To[i+1];
           }
       }
    if ((T_max-T_min) < 5){
       T_max = T_max + 5;
       T_min = T_min - 5;  
    }

    // determine T_center
    // ==================
    T_center = mlx90640To[11* 32 + 15];    

    // drawing the picture
    // ===================

   if (interpolation == 1){
     blur.calculate(mlx90640To, mlx90640ToBlur);
   }
   else {
     interpolate_image(mlx90640To, 24, 32, mlx90640ToBlur, 48, 64);
   }
   
   /*for (i = 0 ; i < 24 ; i++){
        for (j = 0; j < 32; j++){
            mlx90640To[i*32 + j] = 180.0 * (mlx90640To[i*32 + j] - T_min) / (T_max - T_min);                       
            getColour(mlx90640To[i*32 + j]);            
            tft.fillRect(315 - j * 10, 30 + i * 10, 10, 10, tft.color565(R_colour, G_colour, B_colour));
           }
       }
    */
    for (i = 0 ; i < 48 ; i++){
        for (j = 0; j < 64; j++){
            mlx90640ToBlur[i*64 + j] = 180.0 * (mlx90640ToBlur[i*64 + j] - T_min) / (T_max - T_min);                       
            getColour(mlx90640ToBlur[i*64 + j]);            
            tft.fillRect(330 - j * 5, 30 + i * 5, 5, 5, tft.color565(R_colour, G_colour, B_colour));
           }
       }
          
    //draw cross
    tft.drawFastHLine(168, 150, 5, TFT_BLACK);
    tft.drawFastVLine(170, 148, 5, TFT_BLACK);

    tft.setTextColor(TFT_WHITE, tft.color565(0, 0, 0));
    tft.setCursor(275, 300);
    tft.print(T_max, 1);
    tft.setCursor(35, 300);
    tft.print(T_min, 1);
    tft.setCursor(120, 10);
    tft.print(T_center, 1);

    tft.setCursor(300, 300);
    tft.print("C");
    tft.setCursor(60, 300);
    tft.print("C");
    tft.setCursor(145, 10);
    tft.print("C");
    
    drawHisto(T_min, T_max);    
    
    long stopPrintTime = millis();
    
    tft.setCursor(200, 10);
    tft.print("FPS");
    tft.setCursor(220, 10);
    tft.print(1000.0 / (stopPrintTime - startTime), 2);
    
    Serial.print("Read rate: ");
    Serial.print( 1000.0 / (stopReadTime - startTime), 2);
    Serial.println(" Hz");
    Serial.print("Read plus print rate: ");
    Serial.print( 1000.0 / (stopPrintTime - startTime), 2);
    Serial.println(" Hz");
    delay(20);
   }

void drawLegend(){
  // draw scale
  for (int x = 80; x <= 260; x +=30){
    tft.drawFastVLine(x, 300, 10, TFT_WHITE);
  }  
  // drawing the colour-scale
  for (i = 0; i <= colorStep; i++){    
    getColour(i);
    tft.drawLine(80 + i , 300, 80 + i, 310, tft.color565(R_colour, G_colour, B_colour));
    }   
}

void drawHisto(float min, float max){
  float step = (max - min )/colorStep;
  //Serial.println(step, 2);
  for (int  bound = 0; bound < colorStep; bound++){
    b[bound] = min + bound * step;
  } 
  for (int  counter = 0; counter < 768; counter++) {
    hist.add(mlx90640To[counter]);
  }
  for (i = 0; i <= colorStep; i++){    
    getColour(i);
    tft.drawFastVLine(80 + i , 280, 20 - (hist.frequency(i)*250), TFT_BLACK);    
    tft.drawFastVLine(80 + i , 280 + 20 - (hist.frequency(i)*250), (hist.frequency(i)*250) + 1, tft.color565(R_colour, G_colour, B_colour));
    //Serial.println(hist.frequency(i));
  }
  hist.clear(); 
}

// ===============================
// ===== determine the colour ====
// ===============================
void getColour(int j){
    if (j >= 0 && j < 30){
        R_colour = 0;
        G_colour = 0;
        B_colour = 20 + (120.0/30.0) * j;
       }    
    if (j >= 30 && j < 60){
        R_colour = (120.0 / 30) * (j - 30.0);
        G_colour = 0;
        B_colour = 140 - (60.0/30.0) * (j - 30.0);
       }
    if (j >= 60 && j < 90){
        R_colour = 120 + (135.0/30.0) * (j - 60.0);
        G_colour = 0;
        B_colour = 80 - (70.0/30.0) * (j - 60.0);
       }
    if (j >= 90 && j < 120){
        R_colour = 255;
        G_colour = 0 + (60.0/30.0) * (j - 90.0);
        B_colour = 10 - (10.0/30.0) * (j - 90.0);
       }
    if (j >= 120 && j < 150){
        R_colour = 255;
        G_colour = 60 + (175.0/30.0) * (j - 120.0);
        B_colour = 0;
       }
    if (j >= 150 && j <= 180){
        R_colour = 255;
        G_colour = 235 + (20.0/30.0) * (j - 150.0);
        B_colour = 0 + 255.0/30.0 * (j - 150.0);
       }
   }
      
//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected(){
    Wire.beginTransmission((uint8_t)MLX90640_address);
  
    if (Wire.endTransmission() != 0)
       return (false); //Sensor did not ACK
    
    return (true);
   }  

