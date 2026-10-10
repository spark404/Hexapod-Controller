#!/usr/bin/env python3
"""Dynamixel XL430 bus-time model for 18 servos on one half-duplex UART.

Estimates Sync Read / Sync Write time and bus utilization per baud rate for the
scenarios in docs/design.md Appendix A. Timing constants (RDT, GAP, SW) are
assumptions; measure on the robot (logic analyzer on the bus) and adjust.
"""
N=18
BITS=10
RDT=20e-6          # configured Return Delay Time
GAP=10e-6          # extra per-servo inter-status gap (assumption)
SW=60e-6           # STM32 TX->RX / RX->TX direction switch + DMA arm per transaction (assumption)

def sync_read_req(n): return 14+n                 # hdr4 id1 len2 inst1 addr2 len2 ids n crc2
def status(L): return 11+L                         # hdr4 id1 len2 inst1 err1 data L crc2
def sync_write(n,L): return 14+n*(1+L)             # ... addr2 len2 (id+data)*n crc2

def read_time(B, L):
    byte=BITS/B
    return sync_read_req(N)*byte + RDT + N*status(L)*byte + (N-1)*(RDT+GAP) + SW*2

def write_time(B, L):
    return sync_write(N,L)*BITS/B + SW

def line(name, B, rd_hz, rd_L, wr_hz, wr_L):
    tr=read_time(B,rd_L); tw=write_time(B,wr_L)
    busy=tr*rd_hz+tw*wr_hz
    return tr,tw,busy

scen=[
 ("A current: read pos(4B) 50Hz, write goal(4B) 50Hz", 50,4, 50,4),
 ("B design: read load+vel+pos(10B) 50Hz, write profvel+goal(8B) 25Hz", 50,10, 25,8),
 ("C as B, servo loop 100Hz, write 25Hz", 100,10, 25,8),
 ("E load+pos only? read 126..135 100Hz, write 100Hz (worst case)", 100,10, 100,8),
]
bauds=[1e6,2e6,3e6,4e6,4.5e6]
print("Packet sizes @N=18: sync read req %d B; status(4B)=%d B,(10B)=%d B; sync write goal=%d B, vel+goal=%d B"%(
 sync_read_req(N),status(4),status(10),sync_write(N,4),sync_write(N,8)))
for s in scen:
    print("\n"+s[0])
    for B in bauds:
        tr,tw,busy=line(s[0],B,s[1],s[2],s[3],s[4])
        print("  %4.1f Mbps  read %5.2f ms  write %5.2f ms  bus busy %5.1f%%  (one servo cycle read+write %5.2f ms)"%(B/1e6,tr*1e3,tw*1e3,busy*100,(tr+tw)*1e3))
print("\nSTM32F401 USART6 (APB2=84MHz, OVER16): divider and error")
for B in [1e6,2e6,3e6,4e6,4.5e6,5e6]:
    div=84e6/16/B; mant=int(div); frac=round((div-mant)*16)
    if frac==16: mant+=1; frac=0
    act=84e6/16/(mant+frac/16)
    print("  target %.2f M -> BRR %d+%d/16 -> %.4f M (%.2f%% error)"%(B/1e6,mant,frac,act/1e6,(act-B)/B*100))
