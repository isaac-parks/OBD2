// RECEIVE BUFF CONTROL REGISTERS 
#define RXB0CTRL 0x60  // Receive buff 0 control register
#define RXM0 0x6 // RXM0 and RXM1 are control bits for what filters should be applid to receive message - when both are 1, all messages are received
#define RXM1 0x7
#define BUKT 0x3 // Enables rollover to receive register 1 if 0 is already full
// todo rest of bits in register


#define RXB1CTRL 0x70 // Receive buff 1 control register (NOTE: the bits in this register will be different, need to define seperately)
// todo bits in register


// STANDARD IDENTIFIER FOR MESSAGES IN RECEIVE BUFF 0/1 (HIGH BYTE)
#define RXB0SIDH 0x61
#define RXB1SIDH 0x71
// todo bits in register

// STANDARD IDENTIFIER FOR MESSAGES IN RECEIVE BUFF 0/1 (LOW BYTE)
#define RXB0SIDL 0x62
#define RXB1SIDL 0x72
// todo bits in register

// INTERUPT ENABLE REGISTER
#define CANINTE 0x2B
#define RX1IE 0x01 // Whether interrupts are enabled for Receive buff 1
#define RX0IE 0x00 // Whether interrupts are enabled for recieve buff 0
// todo rest of bits in register

// Interrupt flag register
#define CANINTF 0x2C
#define RX0IF 0x01 // first bit in register is Receive buffer 0. If set to 1, an interupt is pending and must be serviced, this app must set it back to 0 after servicing
#define RX1IF 0x02 // Receive buffer 1

// CAN STATUS REGISTER 
#define CANSTAT 0xXE
#define ICOD0 0x01 // First bit is 1 if there is a message in receive buff 0
#define ICOD0 0x02 // Second bit is 1 if there is a message in receive buff 1


enum MODE {
  on,
  off
};

void print_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
      Serial.print((byte >> i) & 0x01);
    }

    Serial.println("");
}


namespace SPI {
  namespace Clock { // Clock control for spi
    int clock_pin = 10;
    int current_clock = LOW;

    int tick() {
      if (current_clock == HIGH) {
        digitalWrite(clock_pin, LOW);
        current_clock = LOW;
      } else {
        digitalWrite(clock_pin, HIGH);
        current_clock = HIGH;
      }

      return current_clock;
    } 

    void init() {
      pinMode(clock_pin, OUTPUT);
      digitalWrite(clock_pin, LOW);
    }
  }


  int chip_select = 13;
  int slave_out = 12;
  int slave_in = 11;
  int interrupt = 9;

  void enable_cs() {
    digitalWrite(chip_select, LOW); // active low
    
  }

  void disable_cs() {
    digitalWrite(chip_select, HIGH); // active low
    
  }

  void slave_out_high_imp() {
    pinMode(slave_out, INPUT);
    
  }

  uint8_t read_slave_out() {
    Clock::tick();
    

    uint8_t bit = digitalRead(slave_out);
    
    Clock::tick();
    return bit;
  }

  uint8_t read_byte() {
    uint8_t byte = 0;
    for (int i = 7; i >= 0; i--) {
        byte |= (read_slave_out() << i);
    }
    
    return byte;
  }

  void write_bit(uint8_t bit) {
    digitalWrite(slave_in, bit);
    
    Clock::tick();
    delayMicroseconds(1);
    Clock::tick();
  }

  void write_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
      write_bit((byte >> i) & 0x01);
    }
  }

  void spi_write(uint8_t addr, uint8_t data) {
    uint8_t write_instruction = 0b00000010;
    slave_out_high_imp();
    enable_cs();
    write_byte(write_instruction);
    write_byte(addr);
    write_byte(data);
    disable_cs();
  }


  uint8_t spi_read(uint8_t addr) {
    uint8_t read_ins = 0b00000011;
    slave_out_high_imp();
    enable_cs();
    write_byte(read_ins);
    write_byte(addr);

    uint8_t data = read_byte();
    disable_cs();
    
    print_byte(data);
    return data;
  }

  void init() {
    pinMode(chip_select, OUTPUT);
    pinMode(slave_out, INPUT);
    pinMode(slave_in, OUTPUT);
    pinMode(interrupt, INPUT);

    disable_cs();

    Clock::init();
  }
}

namespace CANController {
  
  MODE receive_filter_status = off;

  MODE toggle_filters() {
    if (receive_filter_status == on) {
      // write to spi to turn RXB0CTRL off
      receive_filter_status = MODE::off;

      return MODE::off;
    }

    receive_filter_status = on;
    return MODE::on;
  }
}


void setup() {
  Serial.begin(9600);
  SPI::init();
}

void test_write_read(uint8_t addr, uint8_t data) {
  SPI::spi_write(addr, data);
  delay(100);
  SPI::spi_read(addr);
}


uint8_t taddr = 0;
uint8_t tdata = 0;

uint8_t hexCharToByte(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0;
}

uint8_t parseHexByte(const char* hexStr) {
    return (hexCharToByte(hexStr[0]) << 4) + hexCharToByte(hexStr[1]);
}

void test_spi() {
    if (Serial.available() > 0) {
        static char inputBuffer[5];
        static uint8_t inputIndex = 0;
        
        char incomingChar = Serial.read();
        
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (inputIndex == 5) {
                taddr = parseHexByte(inputBuffer);
                tdata = parseHexByte(inputBuffer + 3);
                test_write_read(taddr, tdata);
            }
            inputIndex = 0;
        } else if (inputIndex < 5) {
            inputBuffer[inputIndex++] = incomingChar;
        }
    }
}

void loop() {
  test_spi();
}

// for reading 
// 1. poll the interrupt pin
// 2. when interrupt is received, need to see which buffer the message is in (BFPCTRL)
// 3. need to read the length of the message 


