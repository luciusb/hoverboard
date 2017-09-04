#include "mbed.h"

#define START_BYTE 0xAA
#define UPDATE_TIME_MS 50
#define COM_WD_MS 1000;

volatile uint16_t left, right;
volatile uint16_t timer;
volatile uint8_t enabled;
Ticker tick;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
Serial motor1(PC_12, PD_2);
Serial motor2(PC_10, PC_11);
Serial pc(SERIAL_TX, SERIAL_RX);
DigitalIn encL1(PC_5);
DigitalIn encL2(PC_6);
DigitalIn encL3(PC_8);
DigitalIn encR1(PB_9);
DigitalIn encR2(PB_8);
DigitalIn encR3(PC_9);

volatile uint16_t wd;
volatile uint16_t encL;
volatile uint16_t encR;
volatile uint16_t errL;
volatile uint16_t errR;


enum{ERR=8, ERN=9};
const int8_t enctable [8][8] = {
    // 0 ,  1,   2,   3,   4,   5,   6,   7
    {  0,  -1, ERN,  -2,   1, ERN,   2, ERR},//0
    {  1,   0, ERN,  -1,   2, ERN, ERR,  -2},//1
    {  0,   0,   0,   0,   0, ERN,   0,   0},//2 not a state
    {  2,   1, ERN,   0, ERR, ERN,  -2,  -1},//3
    { -1,  -2, ERN, ERR,   0, ERN,   1,   2},//4
    {  0,   0, ERN,   0,   0,   0,   0,   0},//5 not a state
    { -2, ERR, ERN,   2,  -1, ERN,   0,   1},//6
    {ERR,   2, ERN,   1,  -2, ERN,  -1,   0},//7
};

void writeMotor( Serial * mot, uint16_t speed, uint8_t enabled){
    
    uint8_t sp[2];
    sp[0]=(speed >> 8) & 0xFF;
    sp[1]=speed & 0xFF;

    mot->putc(00);//Start byte
    mot->putc(sp[1]);
    mot->format(8,SerialBase::None,2); //Expects 9bit, 1stop, receives 8bit,2stop -> MSB=first stop bit=1
    mot->putc(sp[0]);
    mot->format(9);
    mot->putc(sp[1]);
    mot->putc(sp[0]);
    if(enabled) mot->putc(0x55);//End byte
    else mot->putc(0x00);//End byte
}

void update()
{
    static uint8_t cnt=0;
    if(wd< UPDATE_TIME_MS)
    {
        left = 0;
        right = 0;
        led1=0;
    }
    else
        wd -= UPDATE_TIME_MS;
    cnt++;
    writeMotor(&motor1, -left, enabled);
    writeMotor(&motor2, right, enabled);
}
typedef enum{ WAIT_FOR_START, READ_LEN, READ_DATA, READ_CHCKSUM} ParseStatus;
void processMsg(uint8_t * buffer, uint16_t len)
{
    if(len>=5)
    {
        left = buffer[0]<<8 | buffer[1];
        right = buffer[2]<<8 | buffer[3];  
        enabled = buffer[4];
    }

}

void parseByte(uint8_t c)
{
    static ParseStatus status = WAIT_FOR_START;
    static uint8_t xorsum = 0;
    static uint8_t len = 0;
    static uint8_t pos = 0;
    static uint8_t buffer [256];
    
    switch (status) {
        case WAIT_FOR_START:
            if(c==START_BYTE)
                status = READ_LEN;
            xorsum = c;
        break;
        case READ_LEN:
            len = c;
            pos = 0;
            xorsum^=c;
            status = READ_DATA;
        break;
        case READ_DATA:
            buffer[pos] = c;
            xorsum^=c;
            pos+=1;
            if(pos>=len)
                status = READ_CHCKSUM;
        break;
        case READ_CHCKSUM:
            if(c == xorsum)
            {
                processMsg(buffer, len);
                //pc.putc('o');
                wd = COM_WD_MS;
                led1=1;
            }
            else
                ;//pc.putc('x');
            status = WAIT_FOR_START;
        break;
    }
}
void rx()
{
    while (pc.readable())
        parseByte(pc.getc());
}
void enc()
{
    timer+=1;
    uint8_t nEncL=(encL1&1)|((encL3&1)<<1)|((encL2&1)<<2);
    uint8_t nEncR=(encR1&1)|((encR3&1)<<1)|((encR2&1)<<2);
    static uint8_t lastEncL =nEncL;
    static uint8_t lastEncR =nEncR;
    int8_t change;
    
    change = enctable[lastEncR][nEncR];
    if(change!=ERN)
        lastEncR=nEncR;
    if(change!=ERR and change!=ERN)
        encR-=change;
    else
        errR++;

    change = enctable[lastEncL][nEncL];
    if(change!=ERN)
        lastEncL=nEncL;
    if(change!=ERR and change!=ERN)
        encL+=change;
    else
        errL++;
}
uint8_t send16(uint16_t n)
{
    pc.putc(n>>8); 
    pc.putc(n&0xff);
    return (n&0xff)^(n>>8);
}
uint8_t send8(uint8_t n)
{
    pc.putc(n); 
    return n;
}

void sendStatus()
{
    timer+=1;
    uint16_t len=12;
    //uint16_t timer = 12345;//TODO
    uint8_t dxor=START_BYTE^len;
    pc.putc(START_BYTE);
    pc.putc(len);
    dxor ^= send16(timer);
    dxor ^= send16(encL);
    dxor ^= send16(encR);
    dxor ^= send16(errL);
    dxor ^= send16(errR);
    dxor ^= send8((encL1&1)|((encL3&1)<<1)|((encL2&1)<<2));
    dxor ^= send8((encR1&1)|((encR3&1)<<1)|((encR2&1)<<2));
    pc.putc(dxor);
}

int main() {
    static uint8_t statuscnt=0;
    pc.baud(115200);
    pc.attach(&rx);
    motor1.baud(26300);
    motor2.baud(26300);
    //enc update on 1ms
    encL=0;
    encR=0;
    timer=0;
    tick.attach(&enc, 1./1000.);
    
    while (1){
            //led1=!led1;
            update();
            wait_ms(UPDATE_TIME_MS);
            statuscnt++;
            if(statuscnt>=5)
            {
                statuscnt=0;
                sendStatus();
            }
    }
    
}
