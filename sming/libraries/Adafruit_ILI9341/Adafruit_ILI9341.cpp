/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
/********************************
 * ported for Sming by M.Bozkurt.
 * alonewolf07@gmail.com
 ********************************/

#include <include/SmingCore.h>
#include "Adafruit_ILI9341.h"
#include <limits.h>
#include <c_types.h>
//#include "sming/core/SPI.h"

#define SWAPBYTES(i) ((i>>8) | (i<<8))
// Constructor when using software SPI.  All output pins are configurable.

#if defined (SPI_HAS_TRANSACTION)
  static SPISettings ILI9341SPISettings;
#elif defined (__AVR__)
  static uint8_t SPCRbackup;
  static uint8_t mySPCR;
#endif


// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ILI9341::Adafruit_ILI9341(int8_t cs, int8_t rs, int8_t sid, int8_t sclk,
        int8_t rst) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
    _cs = cs;
    _rs = rs;
    _sid = sid;
    _sclk = sclk;
    _rst = rst;
    hwSPI = false;
    tabcolor = 0;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9341::Adafruit_ILI9341(int8_t cs, int8_t rs, int8_t rst)
: Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
    _cs = cs;
    _rs = rs;
    _rst = rst;
    hwSPI = true;
    _sid = _sclk = 0;
    tabcolor = 0;
}


inline void Adafruit_ILI9341::spiwrite(uint8_t c) {
    if (hwSPI) {
#if defined (SPI_HAS_TRANSACTION)
        SPI.transfer(c);
#elif defined (__AVR__)
        SPCRbackup = SPCR;
        SPCR = mySPCR;
        SPI.transfer(c);
        SPCR = SPCRbackup;
        //      SPDR = c;
        //      while(!(SPSR & _BV(SPIF)));
#elif defined (__arm__)
        SPI.setClockDivider(21); //4MHz
        SPI.setDataMode(SPI_MODE0);
        SPI.transfer(c);
#elif defined (__ESP8266_EX__)
        SPI.transfer(c);
#endif
    } else {
        // Fast SPI bitbang swiped from LPD8806 library
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            if (c & bit) *dataport |= datapinmask;
            else *dataport &= ~datapinmask;
            *clkport |= clkpinmask;
            *clkport &= ~clkpinmask;
        }
    }
}


void Adafruit_ILI9341::writecommand(uint8_t c) {
#if defined (SPI_HAS_TRANSACTION)
  SPI.beginTransaction(ILI9341SPISettings);
#endif
  *rsport &= ~rspinmask;
  *csport &= ~cspinmask;

  //Serial.print("C ");
  spiwrite(c);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}

void Adafruit_ILI9341::writedata(uint8_t d) {
#if defined (SPI_HAS_TRANSACTION)
    SPI.beginTransaction(ILI9341SPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
    
  //Serial.print("D ");
  spiwrite(d);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}


#define DELAY 0x80
static const uint8_t PROGMEM
Initcmd[] = {                      // Initialization commands for ILI9341 screens
    23,                         // 23 commands in list:
    ILI9341_SWRESET,   DELAY,   //  1: Software reset, no args, w/delay
    50,                         //     50 ms delay
    0xCB, 5,                    //  2: ?
    0x39,0x2C,0x00,0x34,0x02,
    0xCF, 3,                    //  3: ?
    0x00,0XC1,0X30,
    0xE8, 3,                    //  4: ?
    0x85,0x00,0x78,
    0xEA, 2,                    //  5: ?
    0x00,0x00,
    0xED, 4,                    //  6: ?
    0x64,0x03,0X12,0X81,
    0xF7, 1,                    //  7: ?
    0x20,
    0xC0, 1,                    //  8: Power control
    0x23,                       //      VRH[5:0]
    0xC1, 1,                    //  9: Power control
    0x10,                       //      SAP[2:0];BT[3:0]
    0xC5, 2,                    // 10: VCM control
    0x3e,0x28,                  //      Contrast
    0xC7, 1,                    // 11: VCM control2
    0x86,
    0x40, 1,                    // 12:
    0x48,
    0x08, 1,                    // 13:
    0x48,
    ILI9341_PIXFMT, 1,          // 14: ?
    0x55,
    ILI9341_FRMCTR1, 2,         // 15: ?
    0x00,0x18,
    ILI9341_DFUNCTR, 3,         // 16: Display Function Control
    0x08,0x82,0x27,
    0xF2, 1,                    // 17: ?
    0x00,
    ILI9341_GAMMASET, 1,        // 18: Gamma curve selected
    0x01,
    ILI9341_GMCTRP1, 15,        // 19: Set Gamma
    0x0F,0x31,0x2B,0x0C,0x0E,
    0x08,0x4E,0xF1,0x37,0x07,
    0x10,0x03,0x0E,0x09,0x00,
    ILI9341_GMCTRN1, 15,        // 20: Set Gamma
    0x00,0x0E,0x14,0x03,0x11,
    0x07,0x31,0xC1,0x48,0x08,
    0x0F,0x0C,0x31,0x36,0x0F,
    ILI9341_SLPOUT, DELAY,      // 21: Exit Sleep
    120,
    ILI9341_DISPON, 0,          // 22: Display on
    ILI9341_RAMWR, 0};          // 23: ram write
    

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9341::commandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = pgm_read_byte(addr++); // Number of commands to follow
    while (numCommands--) { // For each command...
        writecommand(pgm_read_byte(addr++)); //   Read, issue command
        numArgs = pgm_read_byte(addr++); //   Number of args to follow
        ms = numArgs & DELAY; //   If hibit set, delay follows args
        numArgs &= ~DELAY; //   Mask out delay bit
        while (numArgs--) { //   For each argument...
            writedata(pgm_read_byte(addr++)); //     Read, issue argument
        }
        if (ms) {
            ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
            if (ms == 255) ms = 500; // If 255, delay for 500 ms
            delay(ms);
        }
    }
}


// Set communication using HW/SW SPI Port & Initialization code
void Adafruit_ILI9341::init(uint32_t speed) {
  colstart  = rowstart = 0; // May be overridden in init func

  pinMode(_rs, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  rsport    = portOutputRegister(digitalPinToPort(_rs));
  cspinmask = digitalPinToBitMask(_cs);
  rspinmask = digitalPinToBitMask(_rs);

  if(hwSPI) { // Using hardware SPI
#if defined (SPI_HAS_TRANSACTION)
    SPI.begin();
    ILI9341SPISettings = SPISettings(speed, MSBFIRST, SPI_MODE0);
#elif defined (__AVR__)
    SPCRbackup = SPCR;
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR; // save our preferred state
    //Serial.print("mySPCR = 0x"); Serial.println(SPCR, HEX);
    SPCR = SPCRbackup;  // then restore
#elif defined (__SAM3X8E__)
    SPI.begin();
    SPI.setClockDivider(21); //4MHz
    SPI.setDataMode(SPI_MODE0);
#elif defined (__ESP8266_EX__)
    SPI.begin();

#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
    *clkport   &= ~clkpinmask;
    *dataport  &= ~datapinmask;
  }

  // toggle RST low to reset; CS low so it'll listen to us
  *csport &= ~cspinmask;
  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
    delay(10);
    digitalWrite(_rst, HIGH);
    delay(1);
  }

  commandList(Initcmd);
}




void Adafruit_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

    if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x, y, x + 1, y + 1);

#if defined (SPI_HAS_TRANSACTION)
    SPI.beginTransaction(ILI9341SPISettings);
#endif
    *rsport |= rspinmask;
    *csport &= ~cspinmask;

    spiwrite(color >> 8);
    spiwrite(color);

    *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}
/* void Adafruit_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

    if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x, y, x + 1, y + 1);
    transmitData(SWAPBYTES(color));
}*/

void Adafruit_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
        uint16_t color) {

    // Rudimentary clipping
    if ((x >= _width) || (y >= _height)) return;
    if ((y + h - 1) >= _height) h = _height - y;
    setAddrWindow(x, y, x, y + h - 1);

    uint8_t hi = color >> 8, lo = color;

#if defined (SPI_HAS_TRANSACTION)
    SPI.beginTransaction(ILI9341SPISettings);
#endif
    *rsport |= rspinmask;
    *csport &= ~cspinmask;
    while (h--) {
        spiwrite(hi);
        spiwrite(lo);
    }
    *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}

/*void Adafruit_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
        uint16_t color) {

    // Rudimentary clipping
    if ((x >= _width) || (y >= _height)) return;

    if ((y + h - 1) >= _height)
        h = _height - y;

    setAddrWindow(x, y, x, y + h - 1);
    transmitData(SWAPBYTES(color), h);
}*/


void Adafruit_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
        uint16_t color) {

    // Rudimentary clipping
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    setAddrWindow(x, y, x + w - 1, y);

    uint8_t hi = color >> 8, lo = color;

#if defined (SPI_HAS_TRANSACTION)
    SPI.beginTransaction(ILI9341SPISettings);
#endif
    *rsport |= rspinmask;
    *csport &= ~cspinmask;
    while (w--) {
        spiwrite(hi);
        spiwrite(lo);
    }
    *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}
/*void Adafruit_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
        uint16_t color) {

    // Rudimentary clipping
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    setAddrWindow(x, y, x + w - 1, y);
    transmitData(SWAPBYTES(color), w);
}*/

void Adafruit_ILI9341::fillScreen(uint16_t color) {
    fillRect(0, 0, _width, _height, color);
}

// fill a rectangle

void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color) {

    // rudimentary clipping (drawChar w/big text requires this)
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    if ((y + h - 1) >= _height) h = _height - y;

    setAddrWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8, lo = color;

#if defined (SPI_HAS_TRANSACTION)
    SPI.beginTransaction(ILI9341SPISettings);
#endif
    *rsport |= rspinmask;
    *csport &= ~cspinmask;
    for (y = h; y > 0; y--) {
        for (x = w; x > 0; x--) {
            spiwrite(hi);
            spiwrite(lo);
        }
    }

    *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}
/*void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color) {

    // rudimentary clipping (drawChar w/big text requires this)
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    if ((y + h - 1) >= _height) h = _height - y;

    setAddrWindow(x, y, x + w - 1, y + h - 1);
    transmitData(SWAPBYTES(color), h * w);
}*/



// Pass 8-bit (each) R,G,B, get back 16-bit packed color

uint16_t Adafruit_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341::setRotation(uint8_t r) {
    uint8_t data;
    rotation = r % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            data = MADCTL_MX | MADCTL_BGR;
            _width = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 1:
            data = MADCTL_MV | MADCTL_BGR;
            _width = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
        case 2:
            data = MADCTL_MY | MADCTL_BGR;
            _width = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 3:
            data = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
            _width = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
    }
    writecommand(ILI9341_MADCTL);
    writedata(data);
}

void Adafruit_ILI9341::invertDisplay(bool i) {
    writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}




//void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
//        uint16_t y1) {
//
//    writecommand(ILI9341_CASET); // Column addr set
//    writedata(0x00);
//    writedata(x0 + colstart); // XSTART 
//    writedata(0x00);
//    writedata(x1 + colstart); // XEND
//
//    writecommand(ILI9341_PASET); // Row addr set
//    writedata(0x00);
//    writedata(y0 + rowstart); // YSTART
//    writedata(0x00);
//    writedata(y1 + rowstart); // YEND
//
//    writecommand(ILI9341_RAMWR); // write to RAM
//}

void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
        uint16_t y1) {

    writecommand(ILI9341_CASET); // Column addr set
    writedata((x0 + colstart)>>8);
    writedata(x0 + colstart); // XSTART 
    writedata((x1 + colstart)>>8);
    writedata(x1 + colstart); // XEND

    writecommand(ILI9341_PASET); // Row addr set
    writedata((y0 + rowstart)>>8);
    writedata(y0 + rowstart); // YSTART
    writedata((y1 + rowstart)>>8);
    writedata(y1 + rowstart); // YEND

    writecommand(ILI9341_RAMWR); // write to RAM
}

//MAKEWORD(b1, b2, b3, b4) (uint32_t(b1) | ((b2) << 8) | ((b3) << 16) | ((b4) << 24))
        
//inline void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
//    transmitCmdData(ILI9341_CASET, MAKEWORD(x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF));
//    transmitCmdData(ILI9341_PASET, MAKEWORD(y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF));
//    transmitCmd(ILI9341_RAMWR); // write to RAM
//}
//SWAPED = (num>>8) | (num<<8);

