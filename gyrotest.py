import smbus
from time import sleep
from KalmanFilter import KalmanFilter

def convertData(data0, data1):
    ret = data1 * 256 + data0
    if ret > 32767:
        ret -= 65536
    return ret

# setup smbus
bus = smbus.SMBus(1)

# setup kalman filter
gkfx = KalmanFilter()
gkfx.setAngle(0)

dt = 0.1
ran = 0.0875 
offset = 40
x = 0
prev_x_av = 0
x0 = bus.read_byte_data(0x6b,0x28)
x1 = bus.read_byte_data(0x6b,0x29)
offset = -convertData(x0,x1)*ran
while(1):
    x0 = bus.read_byte_data(0x6b,0x28)
    x1 = bus.read_byte_data(0x6b,0x29)
    y0 = bus.read_byte_data(0x6b,0x2A)
    y1 = bus.read_byte_data(0x6b,0x2B)
    z0 = bus.read_byte_data(0x6b,0x2C)
    z1 = bus.read_byte_data(0x6b,0x2D)
    #x_av = x[0] * ran
    #y_av = y[0] * ran
    #z_av = z[0] * ran

    x_av = convertData(x0,x1)*ran
    y_av = convertData(y0,y1)*ran + offset
    z_av = convertData(z0,z1)*ran + offset

    x = gkfx.calcAngle(x, x_av, dt)

    #x += (prev_x_av + x_av) * 0.1 / 2
    #prev_x_av = x_av

     
    #print("x:{0:+06.2f} y:{0:+06.2f} z:{0:+06.2f}".format(x_av, y_av, z_av))
    print(x)
    sleep(dt)


