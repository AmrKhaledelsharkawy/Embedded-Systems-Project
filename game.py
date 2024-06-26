from machine import Pin, SPI ,UART
from os import uname
import utime

import machine
from machine import I2C
import gc
from machine import Pin, I2C, SPI
import utime
import time
import rp2
import network
import ubinascii
import machine
import time
import socket
try:
    import uasyncio as asyncio
except ImportError:
    import asyncio
import network



class MFRC522:


 
    DEBUG = False
    OK = 0
    NOTAGERR = 1
    ERR = 2
 
    REQIDL = 0x26
    REQALL = 0x52
    AUTHENT1A = 0x60
    AUTHENT1B = 0x61
  
    PICC_ANTICOLL1 = 0x93
    PICC_ANTICOLL2 = 0x95
    PICC_ANTICOLL3 = 0x97
  
 
    def __init__(self, sck, mosi, miso, rst, cs,baudrate=1000000,spi_id=0):
 
        self.sck = Pin(sck, Pin.OUT)
        self.mosi = Pin(mosi, Pin.OUT)
        self.miso = Pin(miso)
        self.rst = Pin(rst, Pin.OUT)
        self.cs = Pin(cs, Pin.OUT)
 
        self.rst.value(0)
        self.cs.value(1)
        
        board = uname()[0]
 
        if board == 'WiPy' or board == 'LoPy' or board == 'FiPy':
            self.spi = SPI(0)
            self.spi.init(SPI.MASTER, baudrate=1000000, pins=(self.sck, self.mosi, self.miso)) # type: ignore
        elif (board == 'esp8266') or (board == 'esp32'):
            # self.spi = SPI(baudrate=100000, polarity=0, phase=0, sck=self.sck, mosi=self.mosi, miso=self.miso) # type: ignore
            # self.spi.init(SPI(baudrate=100000, polarity=0, phase=0, sck=self.sck, mosi=self.mosi, miso=self.miso))
            self.spi.init(SPI.MASTER, baudrate=1000000, polarity=0, phase=0, pins=(self.sck, self.mosi, self.miso)) # type: ignore

        elif board == 'rp2':
            self.spi = SPI(spi_id,baudrate=baudrate,sck=self.sck, mosi= self.mosi, miso= self.miso)
        else:
            raise RuntimeError("Unsupported platform")
 
        self.rst.value(1)
        self.init()
 
    def _wreg(self, reg, val):
 
        self.cs.value(0)
        self.spi.write(b'%c' % int(0xff & ((reg << 1) & 0x7e)))
        self.spi.write(b'%c' % int(0xff & val))
        self.cs.value(1)
 
    def _rreg(self, reg):
 
        self.cs.value(0)
        self.spi.write(b'%c' % int(0xff & (((reg << 1) & 0x7e) | 0x80)))
        val = self.spi.read(1)
        self.cs.value(1)
 
        return val[0]
 
    def _sflags(self, reg, mask):
        self._wreg(reg, self._rreg(reg) | mask)
 
    def _cflags(self, reg, mask):
        self._wreg(reg, self._rreg(reg) & (~mask))
 
    def _tocard(self, cmd, send):
 
        recv = []
        bits = irq_en = wait_irq = n = 0
        stat = self.ERR
 
        if cmd == 0x0E:
            irq_en = 0x12
            wait_irq = 0x10
        elif cmd == 0x0C:
            irq_en = 0x77
            wait_irq = 0x30
 
        self._wreg(0x02, irq_en | 0x80)
        self._cflags(0x04, 0x80)
        self._sflags(0x0A, 0x80)
        self._wreg(0x01, 0x00)
 
        for c in send:
            self._wreg(0x09, c)
        self._wreg(0x01, cmd)
 
        if cmd == 0x0C:
            self._sflags(0x0D, 0x80)
 
        i = 2000
        while True:
            n = self._rreg(0x04)
            i -= 1
            if ~((i != 0) and ~(n & 0x01) and ~(n & wait_irq)):
                break
 
        self._cflags(0x0D, 0x80)
 
        if i:
            if (self._rreg(0x06) & 0x1B) == 0x00:
                stat = self.OK
 
                if n & irq_en & 0x01:
                    stat = self.NOTAGERR
                elif cmd == 0x0C:
                    n = self._rreg(0x0A)
                    lbits = self._rreg(0x0C) & 0x07
                    if lbits != 0:
                        bits = (n - 1) * 8 + lbits
                    else:
                        bits = n * 8
 
                    if n == 0:
                        n = 1
                    elif n > 16:
                        n = 16
 
                    for _ in range(n):
                        recv.append(self._rreg(0x09))
            else:
                stat = self.ERR
 
        return stat, recv, bits
 
    def _crc(self, data):
 
        self._cflags(0x05, 0x04)
        self._sflags(0x0A, 0x80)
 
        for c in data:
            self._wreg(0x09, c)
 
        self._wreg(0x01, 0x03)
 
        i = 0xFF
        while True:
            n = self._rreg(0x05)
            i -= 1
            if not ((i != 0) and not (n & 0x04)):
                break
 
        return [self._rreg(0x22), self._rreg(0x21)]
 
    def init(self):
 
        self.reset()
        self._wreg(0x2A, 0x8D)
        self._wreg(0x2B, 0x3E)
        self._wreg(0x2D, 30)
        self._wreg(0x2C, 0)
        self._wreg(0x15, 0x40)
        self._wreg(0x11, 0x3D)
        self.antenna_on()
 
    def reset(self):
        self._wreg(0x01, 0x0F)
 
    def antenna_on(self, on=True):
 
        if on and ~(self._rreg(0x14) & 0x03):
            self._sflags(0x14, 0x03)
        else:
            self._cflags(0x14, 0x03)
 
    def request(self, mode):
        
        
        self._wreg(0x0D, 0x07)
        (stat, recv, bits) = self._tocard(0x0C, [mode])
 
        if (stat != self.OK) or (bits != 0x10):
            stat = self.ERR
        
        utime.sleep_ms(2000) 

 
        return stat, bits
  
    def anticoll(self,anticolN):
 
        ser_chk = 0
        ser = [anticolN, 0x20]
 
        self._wreg(0x0D, 0x00)
        (stat, recv, bits) = self._tocard(0x0C, ser)
 
        if stat == self.OK:
            if len(recv) == 5:
                for i in range(4):
                    ser_chk = ser_chk ^ recv[i]
                if ser_chk != recv[4]:
                    stat = self.ERR
            else:
                stat = self.ERR
 
        return stat, recv
 
    
    def PcdSelect(self, serNum,anticolN):
        backData = []
        buf = []
        buf.append(anticolN)
        buf.append(0x70)
        #i = 0
        ###xorsum=0;
        for i in serNum:
            buf.append(i)
        #while i<5:
        #    buf.append(serNum[i])
        #    i = i + 1
        pOut = self._crc(buf)
        buf.append(pOut[0])
        buf.append(pOut[1])
        (status, backData, backLen) = self._tocard( 0x0C, buf)
        if (status == self.OK) and (backLen == 0x18):
            return  1
        else:
            return 0
    
    
    def SelectTag(self, uid):
        byte5 = 0
        
        #(status,puid)= self.anticoll(self.PICC_ANTICOLL1)
        #print("uid",uid,"puid",puid)
        for i in uid:
            byte5 = byte5 ^ i
        puid = uid + [byte5]
        
        if self.PcdSelect(puid,self.PICC_ANTICOLL1) == 0:
            return (self.ERR,[])
        return (self.OK , uid)
        
    def tohexstring(self,v):
        s="["
        for i in v:
            if i != v[0]:
                s = s+ ", "
            s=s+ "0x{:02X}".format(i)
        s= s+ "]"
        return s
        
  
            
    
    def SelectTagSN(self):
        valid_uid=[]
        (status,uid)= self.anticoll(self.PICC_ANTICOLL1)
        #print("Select Tag 1:",self.tohexstring(uid))
        if status != self.OK:
            return  (self.ERR,[])
        
        if self.DEBUG:   print("anticol(1) {}".format(uid))
        if self.PcdSelect(uid,self.PICC_ANTICOLL1) == 0:
            return (self.ERR,[])
        if self.DEBUG:   print("pcdSelect(1) {}".format(uid))
        
        #check if first byte is 0x88
        if uid[0] == 0x88 :
            #ok we have another type of card
            valid_uid.extend(uid[1:4])
            (status,uid)=self.anticoll(self.PICC_ANTICOLL2)
            #print("Select Tag 2:",self.tohexstring(uid))
            if status != self.OK:
                return (self.ERR,[])
            if self.DEBUG: print("Anticol(2) {}".format(uid))
            rtn =  self.PcdSelect(uid,self.PICC_ANTICOLL2)
            if self.DEBUG: print("pcdSelect(2) return={} uid={}".format(rtn,uid))
            if rtn == 0:
                return (self.ERR,[])
            if self.DEBUG: print("PcdSelect2() {}".format(uid))
            #now check again if uid[0] is 0x88
            if uid[0] == 0x88 :
                valid_uid.extend(uid[1:4])
                (status , uid) = self.anticoll(self.PICC_ANTICOLL3)
                #print("Select Tag 3:",self.tohexstring(uid))
                if status != self.OK:
                    return (self.ERR,[])
                if self.DEBUG: print("Anticol(3) {}".format(uid))
                if self.PcdSelect(uid,self.PICC_ANTICOLL3) == 0:
                    return (self.ERR,[])
                if self.DEBUG: print("PcdSelect(3) {}".format(uid))
        valid_uid.extend(uid[0:5])
        # if we are here than the uid is ok
        # let's remove the last BYTE whic is the XOR sum
        
        return (self.OK , valid_uid[:len(valid_uid)-1])
        #return (self.OK , valid_uid)
    
    
   
       
    
 
    def auth(self, mode, addr, sect, ser):
        return self._tocard(0x0E, [mode, addr] + sect + ser[:4])[0]
    
    def authKeys(self,uid,addr,keyA=None, keyB=None):
        status = self.ERR
        if keyA is not None:
            status = self.auth(self.AUTHENT1A, addr, keyA, uid)
        elif keyB is not None:
            status = self.auth(self.AUTHENT1B, addr, keyB, uid)
        return status
       
 
    def stop_crypto1(self):
        self._cflags(0x08, 0x08)
 
    def read(self, addr):
 
        data = [0x30, addr]
        data += self._crc(data)
        (stat, recv, _) = self._tocard(0x0C, data)
        return stat, recv
 
    def write(self, addr, data):
 
        buf = [0xA0, addr]
        buf += self._crc(buf)
        (stat, recv, bits) = self._tocard(0x0C, buf)
 
        if not (stat == self.OK) or not (bits == 4) or not ((recv[0] & 0x0F) == 0x0A):
            stat = self.ERR
        else:
            buf = []
            for i in range(16):
                buf.append(data[i])
            buf += self._crc(buf)
            (stat, recv, bits) = self._tocard(0x0C, buf)
            if not (stat == self.OK) or not (bits == 4) or not ((recv[0] & 0x0F) == 0x0A):
                stat = self.ERR
        return stat
 
 
    def writeSectorBlock(self,uid, sector, block, data, keyA=None, keyB = None):
        absoluteBlock =  sector * 4 + (block % 4)
        if absoluteBlock > 63 :
            return self.ERR
        if len(data) != 16:
            return self.ERR
        if self.authKeys(uid,absoluteBlock,keyA,keyB) != self.ERR :
            return self.write(absoluteBlock, data)
        return self.ERR
 
    def readSectorBlock(self,uid ,sector, block, keyA=None, keyB = None):
        absoluteBlock =  sector * 4 + (block % 4)
        if absoluteBlock > 63 :
            return self.ERR, None
        if self.authKeys(uid,absoluteBlock,keyA,keyB) != self.ERR :
            return self.read(absoluteBlock)
        return self.ERR, None
 
    def MFRC522_DumpClassic1K(self,uid, Start=0, End=64, keyA=None, keyB=None):
        for absoluteBlock in range(Start,End):
            status = self.authKeys(uid,absoluteBlock,keyA,keyB)
            # Check if authenticated
            print("{:02d} S{:02d} B{:1d}: ".format(absoluteBlock, absoluteBlock//4 , absoluteBlock % 4),end="")
            if status == self.OK:                    
                status, block = self.read(absoluteBlock)
                if status == self.ERR:
                    break
                else:
                    for value in block:
                        print("{:02X} ".format(value),end="")
                    print("  ",end="")
                    for value in block:
                        if (value > 0x20) and (value < 0x7f):
                            print(chr(value),end="")
                        else:
                            print('.',end="")
                    print("")
            else:
                break
            if status == self.ERR:
                print("Authentication error")
                return self.ERR
        return self.OK



"""Provides an API for talking to HD44780 compatible character LCDs."""


class LcdApi:
    """Implements the API for talking with HD44780 compatible character LCDs.
    This class only knows what commands to send to the LCD, and not how to get
    them to the LCD.

    It is expected that a derived class will implement the hal_xxx functions.
    """

    # The following constant names were lifted from the avrlib lcd.h
    # header file, however, I changed the definitions from bit numbers
    # to bit masks.
    #
    # HD44780 LCD controller command set

    LCD_CLR = 0x01              # DB0: clear display
    LCD_HOME = 0x02             # DB1: return to home position

    LCD_ENTRY_MODE = 0x04       # DB2: set entry mode
    LCD_ENTRY_INC = 0x02        # --DB1: increment
    LCD_ENTRY_SHIFT = 0x01      # --DB0: shift

    LCD_ON_CTRL = 0x08          # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY = 0x04       # --DB2: turn display on
    LCD_ON_CURSOR = 0x02        # --DB1: turn cursor on
    LCD_ON_BLINK = 0x01         # --DB0: blinking cursor

    LCD_MOVE = 0x10             # DB4: move cursor/display
    LCD_MOVE_DISP = 0x08        # --DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT = 0x04       # --DB2: move right (0-> left)

    LCD_FUNCTION = 0x20         # DB5: function set
    LCD_FUNCTION_8BIT = 0x10    # --DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # --DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # --DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET = 0x30   # See "Initializing by Instruction" section

    LCD_CGRAM = 0x40            # DB6: set CG RAM address
    LCD_DDRAM = 0x80            # DB7: set DD RAM address

    LCD_RS_CMD = 0
    LCD_RS_DATA = 1

    LCD_RW_WRITE = 0
    LCD_RW_READ = 1

    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        if self.num_lines > 4:
            self.num_lines = 4
        self.num_columns = num_columns
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        self.implied_newline = False
        self.backlight = True
        self.display_off()
        self.backlight_on()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

    def show_cursor(self):
        """Causes the cursor to be made visible."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def hide_cursor(self):
        """Causes the cursor to be hidden."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        """Turns on the cursor, and makes it blink."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        """Turns on the cursor, and makes it no blink (i.e. be solid)."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def display_on(self):
        """Turns on (i.e. unblanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        """Turns off (i.e. blanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL)

    def backlight_on(self):
        """Turns the backlight on.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = True
        self.hal_backlight_on()

    def backlight_off(self):
        """Turns the backlight off.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = False
        self.hal_backlight_off()

    def move_to(self, cursor_x, cursor_y):
        """Moves the cursor position to the indicated position. The cursor
        position is zero based (i.e. cursor_x == 0 indicates first column).
        """
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:    # Lines 2 & 3 add number of columns
            addr += self.num_columns
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        """Writes the indicated character to the LCD at the current cursor
        position, and advances the cursor by one position.
        """
        if char == '\n':
            if self.implied_newline:
                # self.implied_newline means we advanced due to a wraparound,
                # so if we get a newline right after that we ignore it.
                self.implied_newline = False
            else:
                self.cursor_x = self.num_columns
        else:
            self.hal_write_data(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y += 1
            self.implied_newline = (char != '\n')
        if self.cursor_y >= self.num_lines:
            self.cursor_y = 0
        self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        """Write the indicated string to the LCD at the current cursor
        position and advances the cursor position appropriately.
        """
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        self.hal_sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            self.hal_sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_write_command(self, cmd):
        """Write a command to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_write_data(self, data):
        """Write data to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    # This is a default implementation of hal_sleep_us which is suitable
    # for most micropython implementations. For platforms which don't
    # support `time.sleep_us()` they should provide their own implementation
    # of hal_sleep_us in their hal layer and it will be used instead.
    def hal_sleep_us(self, usecs):
        """Sleep for some time (given in microseconds)."""
        time.sleep_us(usecs)  # NOTE this is not part of Standard Python library, specific hal layers will need to override this

html_form = """<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<style>
/* Overall body styling */
body {
  font-family: 'Arial', sans-serif;
  background-color: #f4f4f4;
  margin: 0;
  padding: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100vh;
  color: #333;
}

/* Styling for the game canvas */
canvas {
  border: 2px solid #555;
  background-color: #fff;
  margin-top: 20px;
}

/* Start form styling */
#startForm {
  text-align: center;
  padding: 20px;
  background-color: #ffffff;
  border-radius: 10px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
  margin-bottom: 20px;
}

#startForm input[type="text"] {
  padding: 10px;
  margin-bottom: 10px;
  width: 80%;
  border: 2px solid #ddd;
  border-radius: 5px;
  font-size: 16px;
}

#startForm button {
  padding: 10px 20px;
  background-color: #4caf50;
  color: white;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 16px;
  transition: background-color 0.3s;
}

/* Button hover effect */
#startForm button:hover {
  background-color: #45a049;
}

/* Game controls styling */
.game {
  text-align: center;
}

.game button {
  padding: 10px 20px;
  background-color: #008CBA;
  color: white;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 16px;
  margin-top: 20px;
  transition: background-color 0.3s, transform 0.2s;
}

.game button:hover {
  background-color: #007B9E;
  transform: scale(1.05);
}

canvas {
  border: 1px solid #d3d3d3;
  background-color: #f1f1f1;
}
#startForm {
  text-align: center;
  margin-top: 20px;
}
</style>
</head>
<body>
<div id="startForm">

  <input type="text" id="playerName" placeholder="Enter your name" name="name" required>
  <button onclick="startGame()">Start Game</button>


 </div>

<script>
var myGamePiece;
var myObstacles = [];
var myScore;
var playerName = '';
var gameDiv = document.querySelector('.game');


function startGame() {
    playerName = document.getElementById('playerName').value;
    document.getElementById('startForm').style.display = 'none';
    
    myGamePiece = new component(30, 30, "red", 10, 120);
    myGamePiece.gravity = 0.05;
    myScore = new component("30px", "Consolas", "black", 280, 40, "text");
    myGameArea.start();
}

var myGameArea = {
    canvas: document.createElement("canvas"),
    start: function() {
        this.canvas.width = 480;
        this.canvas.height = 270;
        this.context = this.canvas.getContext("2d");
        document.body.insertBefore(this.canvas, document.body.childNodes[0]);
        this.frameNo = 0;
        this.interval = setInterval(updateGameArea, 20);
    },
    clear: function() {
        this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
    },
    stop: function() {
        clearInterval(this.interval);
        document.body.innerHTML += '<p>Hard luck, ' + playerName + '! </p><button onclick="window.location.reload()">Play Again</button>';
    }
}

function component(width, height, color, x, y, type) {
    this.type = type;
    this.width = width;
    this.height = height;
    this.speedX = 0;
    this.speedY = 0;
    this.x = x;
    this.y = y;
    this.gravity = 0;
    this.gravitySpeed = 0;
    this.update = function() {
        ctx = myGameArea.context;
        if (this.type == "text") {
            ctx.font = this.width + " " + this.height;
            ctx.fillStyle = color;
            ctx.fillText(this.text, this.x, this.y);
        } else {
            ctx.fillStyle = color;
            ctx.fillRect(this.x, this.y, this.width, this.height);
        }
    }
    this.newPos = function() {
        this.gravitySpeed += this.gravity;
        this.x += this.speedX;
        this.y += this.speedY + this.gravitySpeed; 
        this.hitBottom();
    }
    this.hitBottom = function() {
        var rockbottom = myGameArea.canvas.height - this.height;
        if (this.y > rockbottom) {
            this.y = rockbottom;
            this.gravitySpeed = 0;
        }
    }
    this.crashWith = function(otherobj) {
        var myleft = this.x;
        var myright = this.x + (this.width);
        var mytop = this.y;
        var mybottom = this.y + (this.height);
        var otherleft = otherobj.x;
        var otherright = otherobj.x + (otherobj.width);
        var othertop = otherobj.y;
        var otherbottom = otherobj.y + (otherobj.height);
        var crash = true;
        if ((mybottom < othertop) ||
                (mytop > otherbottom) ||
                (myright < otherleft) ||
                (myleft > otherright)) {
            crash = false;
        }
        return crash;
    }
}

function updateGameArea() {
    var x, height, gap, minHeight, maxHeight, minGap, maxGap;
    for (i = 0; i < myObstacles.length; i += 1) {
        if (myGamePiece.crashWith(myObstacles[i])) {
            myGameArea.stop();
            var xhr = new XMLHttpRequest();
        xhr.open("POST", "/submit-score", true);
        xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
        xhr.send("name=" + encodeURIComponent(playerName) + "&score=" + myGameArea.frameNo);
            gameDiv.style.display = 'block';
            return;
        }
    }
    myGameArea.clear();
    myGameArea.frameNo += 1;
    if (myGameArea.frameNo == 1 || everyinterval(150)) {
        x = myGameArea.canvas.width;
        minHeight = 20;
        maxHeight = 200;
        height = Math.floor(Math.random()*(maxHeight-minHeight+1)+minHeight);
        minGap = 50;
        maxGap = 200;
        gap = Math.floor(Math.random()*(maxGap-minGap+1)+minGap);
        myObstacles.push(new component(10, height, "green", x, 0));
        myObstacles.push(new component(10, x - height - gap, "green", x, height + gap));
    }
    for (i = 0; i < myObstacles.length; i += 1) {
        myObstacles[i].x += -1;
        myObstacles[i].update();
    }
    myScore.text="SCORE: " + myGameArea.frameNo;
    myScore.update();
    myGamePiece.newPos();
    myGamePiece.update();
}

function everyinterval(n) {
    if ((myGameArea.frameNo / n) % 1 == 0) {return true;}
    return false;
}

function accelerate(n) {
    myGamePiece.gravity = n;
}
</script>

<br>
<div class="game">
<button onmousedown="accelerate(-0.2)" onmouseup="accelerate(0.05)">ACCELERATE</button>
<p>Use the ACCELERATE button to stay in the air</p>
<p>How long can you stay alive?</p>
</div>
</body>
</html>  

 """

# PCF8574 pin definitions
MASK_RS = 0x01       # P0
MASK_RW = 0x02       # P1
MASK_E  = 0x04       # P2

SHIFT_BACKLIGHT = 3  # P3
SHIFT_DATA      = 4  # P4-P7
# LCD and RFID settings
I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

class I2cLcd(LcdApi):
    
    #Implements a HD44780 character LCD connected via PCF8574 on I2C

    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.i2c.writeto(self.i2c_addr, bytes([0]))
        utime.sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        utime.sleep_ms(5)    # Need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        utime.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        utime.sleep_ms(1)
        # Put LCD into 4-bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        utime.sleep_ms(1)
        LcdApi.__init__(self, num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)
        gc.collect()

    def hal_write_init_nibble(self, nibble):
        # Writes an initialization nibble to the LCD.
        # This particular function is only used during initialization.
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.i2c.writeto(self.i2c_addr, bytes([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))
        gc.collect()
        
    def hal_backlight_on(self):
        # Allows the hal layer to turn the backlight on
        self.i2c.writeto(self.i2c_addr, bytes([1 << SHIFT_BACKLIGHT]))
        gc.collect()
        
    def hal_backlight_off(self):
        #Allows the hal layer to turn the backlight off
        self.i2c.writeto(self.i2c_addr, bytes([0]))
        gc.collect()
        
    def hal_write_command(self, cmd):
        # Write a command to the LCD. Data is latched on the falling edge of E.
        byte = ((self.backlight << SHIFT_BACKLIGHT) |
                (((cmd >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytes([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))
        byte = ((self.backlight << SHIFT_BACKLIGHT) |
                ((cmd & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytes([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))
        if cmd <= 3:
            # The home and clear commands require a worst case delay of 4.1 msec
            utime.sleep_ms(5)
        gc.collect()

    def hal_write_data(self, data):
        # Write data to the LCD. Data is latched on the falling edge of E.
        byte = (MASK_RS |
                (self.backlight << SHIFT_BACKLIGHT) |
                (((data >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytes([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))
        byte = (MASK_RS |
                (self.backlight << SHIFT_BACKLIGHT) |
                ((data & 0x0f) << SHIFT_DATA))      
        self.i2c.writeto(self.i2c_addr, bytes([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))
        gc.collect()

#RFID and LCD
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)
rfid_reader = MFRC522(spi_id=0, sck=2, miso=4, mosi=7, cs=5, rst=18)
variable_to_display = 2  # Initial value for available spots


#Comunication Pins 
comm_send_pin = Pin(21, Pin.OUT)
comm_recv_pin = Pin(22, Pin.IN, Pin.PULL_DOWN)
comm_conf2_pin = Pin(13, Pin.IN, Pin.PULL_DOWN)
comm_conf_pin = Pin(27,  Pin.OUT)
comm_conf3_pin = Pin(16,  Pin.OUT)
comm_conf4_pin = Pin(15,  Pin.IN,Pin.PULL_DOWN)


#Button
Button = Pin(12,Pin.IN,Pin.PULL_UP)
state=1
StateLED_Red = Pin(9,Pin.OUT)
StateLED_Green = Pin(9,Pin.OUT)
StateLED_Blue = Pin(9,Pin.OUT)

def get_button():
    return not Button.value()
def display_variable(x):
    lcd.clear()
    lcd.putstr("Spots Remaining: {}".format(x))
def printLCD(x):
    lcd.clear()
    lcd.putstr(x)
    utime.sleep(1)



def url_decode(s):
    """
    A simple URL decode function for basic decoding needs in MicroPython.
    """
    s = s.replace("+", " ")  # Replace + with space
    s = s.replace("%20", " ")  # Replace %20 with space
    # Add more replacements as needed for your application
    return s

def handle_request(client_socket):
    request_data = client_socket.recv(1024).decode('utf-8')
    request_method = request_data.split(' ')[0]
    if request_method == "POST":
        # Extract the last line of the request data which contains the form data
        form_data = request_data.split('\r\n')[-1]

        # Split the form data into name and score
        form_items = form_data.split('&')
        name_data = form_items[0].split('=')[1]
        score_data = form_items[1].split('=')[1] if len(form_items) > 1 else '0'

        # Split and re-join the name to replace '%20' with spaces
        namelist = name_data.split('%20')
        lastname = " ".join(namelist)

        # Optional: Convert score to integer if necessary
        score = int(score_data)

        # Display the name and score on the LCD (assuming printLCD is your display function)
        printLCD(f"{lastname}: {score}")

        # Redirect back to the game after submitting the score
        response = "HTTP/1.1 303 See Other\r\nLocation: /\r\n\r\n"
        client_socket.send(response.encode('utf-8'))

        
    else:  # Assuming it's a GET request
        # Send the initial HTML form
        response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + html_form
        client_socket.send(response.encode('utf-8'))

    client_socket.close()

def website(state1):
    # Configure the WiFi connection
    ssid = "12345678"
    password = "12345678"
    printLCD("Connecting To the WIFI")
    utime.sleep(1)
    station = network.WLAN(network.STA_IF)  # Use network.WLAN(network.STA_IF) instead of network.STA_IF
    utime.sleep(1)
    station.active(True)
    station.connect(ssid, password)

    # Wait for the connection to be established
    while not station.isconnected():
        pass

    # Print to LCD instead of console
    printLCD("Connected to WiFi")
    ip_address = station.ifconfig()[0]
    printLCD(f"Pico IP address: {ip_address}")

    # Create and start the web server
    port = 80
    server_socket = socket.socket()
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(5)

    # Print to LCD instead of console
    printLCD(f"Web server started on port {port}")
    printLCD(f"Connect To:  {ip_address}")
    utime.sleep(2)
   
                        

    try:
        while True:
            client_socket, addr = server_socket.accept()
            handle_request(client_socket)
       
           
                
    except KeyboardInterrupt:
        pass

    server_socket.close()


    # Print to LCD instead of console
    printLCD("Web server stopped")


def main():
    state1 =2
    while True:
         website(state1)


main()