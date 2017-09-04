from serial import Serial, SerialException
from time import sleep
import msvcrt
#10 revolutions = 864 ticks

def xor(buffer):
    result=0
    for x in buffer:
        result^=x
    return result

class Hoverboard:
    def __init__(self, com):
        self.com = Serial(port=com, baudrate=115000, timeout=0)
        self.com.flushInput()
        self.com.flushOutput()
        self.sendSpeed(0, 0, False)
        self.msgs=[]
        self.rxbuffer=[]

    def sendMsg(self, msg):
        buffer=[0xAA,len(msg)]+msg
        buffer+=[xor(buffer)]
        self.com.write(''.join([chr(x) for x in buffer]))

    def sendSpeed(self, m1 , m2, enabled=True):
        if m1<0:
            m1+=0xffff
        if m2<0:
            m2+=0xffff
        self.sendMsg([(m1>>8)&0xff, m1&0xff, (m2>>8)&0xff, m2&0xff, 1 if enabled else 0 ])

    def processchar(self, c):
        self.rxbuffer.append(c)
        while self.rxbuffer and self.rxbuffer[0]!=0xAA:
            self.rxbuffer.pop(0)
        if len(self.rxbuffer)>=2 and len(self.rxbuffer)>=3+self.rxbuffer[1]:
            msg=self.rxbuffer[:2+self.rxbuffer[1]]
            self.rxbuffer=self.rxbuffer[2+self.rxbuffer[1]:]
            mxor=self.rxbuffer.pop(0)
            cxor=xor(msg)
            return msg[2:], cxor == mxor
        return [], False


    def readMsg(self):
        for c in self.com.read():
            msg, xok=self.processchar(ord(c))
            if msg:
                if xok:
                    self.msgs.append(msg)
                else:
                    print("incomplete msg", msg)
        if self.msgs:
            return self.msgs.pop(0)
        else:
            return None


    def drive(self, s1, s2, t):
        for i in range(int(t/0.1)):
            self.sendSpeed(s1, s2, enabled = True)
            print(h.readMsg())
            sleep(0.1)


h = Hoverboard(com=6)
sleep(3)
acc=100
s1,s2=0,0
keymap={'q':(0,1), 'w':(1,0), 'a':(0,-1), 's':(-1,0)}
try:
    while True:
        while msvcrt.kbhit():
            c=msvcrt.getch()
            if ord(c)==27:
                s1,s2=0,0
            elif c in keymap:
                s1,s2 = s1+acc*keymap[c][0], s2+acc*keymap[c][1]
        h.sendSpeed(s1,s2,True)
        #print s2,s1
        msg=h.readMsg()
        while not msg:
            sleep(0.01)
            msg=h.readMsg()
        vmsg=msg
        while vmsg:
            msg=vmsg
            vmsg=h.readMsg()
        timer=msg[0]<<8|msg[1]
        encL=msg[2]<<8|msg[3]
        encR=msg[4]<<8|msg[5]
        errL=msg[6]<<8|msg[7]
        errR=msg[8]<<8|msg[9]
        encLs=msg[10]
        encRs=msg[11]
        #print timer, "%5s, %5s"%(bin(encL|(1<<4)),bin(encR|(1<<4)))
        print timer, encL, encR, errL, errR, "%5s, %5s"%(bin(encLs|(1<<4)),bin(encRs|(1<<4)))

except KeyboardInterrupt:
    pass
h.sendSpeed(0, 0, False)

