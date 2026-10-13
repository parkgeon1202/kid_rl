# EBIMU quaternion_gyro 형식(*qz,qy,qx,qw,gx,gy,gz\r\n)을 100 Hz 로 흘린다.
# 자세는 수평(단위 쿼터니언), 자이로는 천천히 변하는 사인파라 관측이 틱마다 달라진다.
import math, sys, time
port = open(sys.argv[1], "wb", buffering=0)
t0 = time.time()
while True:
    t = time.time() - t0
    gx, gy, gz = 20 * math.sin(2.1 * t), 15 * math.sin(1.3 * t + 1), 10 * math.sin(0.7 * t + 2)
    port.write(f"*0.0000,0.0000,0.0000,1.0000,{gx:.2f},{gy:.2f},{gz:.2f}\r\n".encode())
    time.sleep(0.01)
