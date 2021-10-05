#!/usr/bin/python
import numpy as np


class Edge:
    def __init__(self, t, rising = True):
        self.t = t
        self.rising = rising

class Device:
    def __init__(self, freq, pw, rep = 1, idle = 0.0):
        self.freq = freq
        self.pw = pw
        self.rep = rep
        self.idle = idle
        self.data = []
    
    def simulate(self, duration, t_offset):
        self.data = []
        subperiod = self.pw + self.idle
        for t in np.arange(0.0 + t_offset, duration + t_offset, 1.0 / self.freq):
            for j in range(self.rep):
                self.data.append(Edge(t + subperiod * j, True))
                self.data.append(Edge(t + subperiod * j + self.pw, False))


def calcOverlappingRatio(da, db):
    if not da.data or not db.data:
        raise RuntimeError('Empty waveform')
    t0 = min(da.data[0].t, db.data[0].t)
    duration_overlap = 0
    va = False
    vb = False
    t_prev = t0
    ia = 0
    ib = 0
    while ia < len(da.data) and ib < len(db.data):
        if da.data[ia].t < db.data[ib].t:
            t_advance = da.data[ia].t - t_prev
            if va and vb:
                duration_overlap += t_advance
            t_prev = da.data[ia].t
            va = da.data[ia].rising
            ia += 1
        else:
            t_advance = db.data[ib].t - t_prev
            if va and vb:
                duration_overlap += t_advance
            t_prev = db.data[ib].t
            vb = db.data[ib].rising
            ib += 1
    return duration_overlap / (t_prev - t0)
        
        
        

def main():
    optitrack = Device(210.0, 250.0 / 1e6)
    kinect = Device(30.0, 125.0 / 1e6, 9, 1450 / 1e6)

    optitrack.simulate(10.0, 0.001)
    kinect.simulate(10.0, 0.0)

    print(calcOverlappingRatio(optitrack, kinect))
    

if __name__ == "__main__":
    main()