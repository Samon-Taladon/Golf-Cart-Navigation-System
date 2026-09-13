# # configure_f9r.py — ใช้กับ u-blox F9R
# import serial
# import time
# from pyubx2 import UBXMessage, SET

# PORT = '/dev/ttyACM0'
# BAUD = 38400

# def send(ser, msg):
#     ser.write(msg.serialize())
#     time.sleep(0.2)
#     print(f"✅ {msg.identity}")

# with serial.Serial(PORT, BAUD, timeout=2) as ser:

#     print("🚀 Start configuring F9R...\n")

#     # =========================
#     # 1. เปิด GNSS ทุกระบบ
#     # =========================
#     gnss = UBXMessage('CFG','CFG-GNSS', SET,
#         msgVer=0, numTrkChHw=0, numTrkChUse=0, numConfigBlocks=4,

#         gnssId_01=0, resTrkCh_01=8, maxTrkCh_01=16, flags_01=0x01000001, # GPS
#         gnssId_02=6, resTrkCh_02=4, maxTrkCh_02=12, flags_02=0x01000001, # GLONASS
#         gnssId_03=2, resTrkCh_03=4, maxTrkCh_03=8,  flags_03=0x01000001, # Galileo
#         gnssId_04=3, resTrkCh_04=4, maxTrkCh_04=12, flags_04=0x01000001, # BeiDou
#     )
#     send(ser, gnss)

#     # =========================
#     # 2. ตั้ง Navigation Model
#     # =========================
#     nav5 = UBXMessage('CFG','CFG-NAV5', SET,
#         mask=0x0005,
#         dynModel=4,   # Automotive
#         minElev=15,
#     )
#     send(ser, nav5)

#     # =========================
#     # 3. ตั้ง Output Rate (10Hz)
#     # =========================
#     rate = UBXMessage('CFG','CFG-RATE', SET,
#         measRate=100,   # 100 ms = 10Hz
#         navRate=1,
#         timeRef=0       # UTC
#     )
#     send(ser, rate)

#     # =========================
#     # 4. เปิดข้อความสำคัญ (ROS2 ใช้)
#     # =========================
#     # NAV-PVT
#     nav_pvt = UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x07,
#         rateUART1=1
#     )
#     send(ser, nav_pvt)

#     # NAV-SAT
#     nav_sat = UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x35,
#         rateUART1=1
#     )
#     send(ser, nav_sat)

#     # =========================
#     # 5. Save config
#     # =========================
#     save = UBXMessage('CFG','CFG-CFG', SET,
#         clearMask=b'\x00\x00\x00\x00',
#         saveMask=b'\x1F\x1F\x00\x00',
#         loadMask=b'\x00\x00\x00\x00',
#         deviceMask=b'\x07\x00\x00\x00'
#     )
#     send(ser, save)

#     print("\n💾 Config saved")

#     # =========================
#     # 6. Software Reset
#     # =========================
#     print("\n🔄 Resetting device...\n")

#     reset = UBXMessage('CFG','CFG-RST', SET,
#         navBbrMask=0xFFFF,
#         resetMode=0x01  # controlled software reset
#     )
#     send(ser, reset)

#     print("🎉 DONE!")
#     print("👉 รอ 3-5 วินาที แล้วใช้งานต่อได้เลย\n")


# configure_f9r.py — tuned for stability (ROS2 + EKF ready)







import serial
import time
from pyubx2 import UBXMessage, SET

PORT = '/dev/ttyACM0'
BAUD = 38400

def send(ser, msg):
    try:
        ser.write(msg.serialize())
        time.sleep(0.4)  # เพิ่ม delay กันหลุด
        print(f"✅ {msg.identity}")
    except Exception as e:
        print(f"⚠️ Error: {e}")
        print("🔄 Reconnecting...")
        try:
            ser.close()
            time.sleep(2)
            ser.open()
            time.sleep(1)
            ser.write(msg.serialize())
            print(f"✅ (retry) {msg.identity}")
        except Exception as e2:
            print(f"❌ Failed again: {e2}")

with serial.Serial(PORT, BAUD, timeout=2) as ser:

    print("🚀 Start configuring F9R (UBX ONLY MODE)...\n")

    # =========================
    # 0. ปิด NMEA ทั้งหมดก่อน
    # =========================
    print("🛑 Disabling NMEA messages...\n")

    nmea_msgs = [
        (0xF0, 0x00), # GGA
        (0xF0, 0x01), # GLL
        (0xF0, 0x02), # GSA
        (0xF0, 0x03), # GSV
        (0xF0, 0x04), # RMC
        (0xF0, 0x05), # VTG
        (0xF0, 0x06), # GRS
        (0xF0, 0x07), # GST
        (0xF0, 0x08), # ZDA
    ]

    for cls, mid in nmea_msgs:
        send(ser, UBXMessage('CFG','CFG-MSG', SET,
            msgClass=cls,
            msgID=mid,
            rateUART1=0
        ))

    # =========================
    # 1. GNSS (ลด noise)
    # =========================
    print("\n🛰️ Config GNSS...\n")

    gnss = UBXMessage('CFG','CFG-GNSS', SET,
        msgVer=0, numTrkChHw=0, numTrkChUse=0, numConfigBlocks=3,

        # GPS
        gnssId_01=0, resTrkCh_01=8, maxTrkCh_01=16, flags_01=0x01000001,

        # GLONASS
        gnssId_02=6, resTrkCh_02=4, maxTrkCh_02=12, flags_02=0x01000001,

        # Galileo
        gnssId_03=2, resTrkCh_03=4, maxTrkCh_03=8, flags_03=0x01000001,
    )
    send(ser, gnss)

    # =========================
    # 2. Navigation Model
    # =========================
    nav5 = UBXMessage('CFG','CFG-NAV5', SET,
        mask=0x0005,
        dynModel=4,
        minElev=20,
    )
    send(ser, nav5)

    # =========================
    # 3. Output Rate (5Hz)
    # =========================
    rate = UBXMessage('CFG','CFG-RATE', SET,
        measRate=200,
        navRate=1,
        timeRef=0
    )
    send(ser, rate)

    # =========================
    # 4. เปิด UBX Messages
    # =========================
    print("\n📡 Enabling UBX messages...\n")

    # NAV-PVT
    send(ser, UBXMessage('CFG','CFG-MSG', SET,
        msgClass=0x01, msgID=0x07, rateUART1=1))

    # NAV-SAT
    send(ser, UBXMessage('CFG','CFG-MSG', SET,
        msgClass=0x01, msgID=0x35, rateUART1=1))

    # NAV-VELNED
    send(ser, UBXMessage('CFG','CFG-MSG', SET,
        msgClass=0x01, msgID=0x12, rateUART1=1))

    # NAV-HPPOSLLH
    send(ser, UBXMessage('CFG','CFG-MSG', SET,
        msgClass=0x01, msgID=0x14, rateUART1=1))

    # =========================
    # 5. Save config
    # =========================
    print("\n💾 Saving config...\n")

    save = UBXMessage('CFG','CFG-CFG', SET,
        clearMask=b'\x00\x00\x00\x00',
        saveMask=b'\x1F\x1F\x00\x00',
        loadMask=b'\x00\x00\x00\x00',
        deviceMask=b'\x07\x00\x00\x00'
    )
    send(ser, save)

    # =========================
    # 6. Reset
    # =========================
    print("\n🔄 Resetting device...\n")

    reset = UBXMessage(
        'CFG','CFG-RST', SET,
        navBbrMask=0xFFFF,
        resetMode=0x01
    )

    send(ser, reset)

    print("✅ Reset command sent")






# import serial
# import time
# from pyubx2 import UBXMessage, SET

# PORT = '/dev/ttyACM0'
# BAUD = 38400

# def send(ser, msg):
#     try:
#         ser.write(msg.serialize())
#         time.sleep(0.4)  # เพิ่ม delay กันหลุด
#         print(f"✅ {msg.identity}")
#     except Exception as e:
#         print(f"⚠️ Error: {e}")
#         print("🔄 Reconnecting...")
#         try:
#             ser.close()
#             time.sleep(2)
#             ser.open()
#             time.sleep(1)
#             ser.write(msg.serialize())
#             print(f"✅ (retry) {msg.identity}")
#         except Exception as e2:
#             print(f"❌ Failed again: {e2}")

# with serial.Serial(PORT, BAUD, timeout=2) as ser:

#     print("🚀 Start configuring F9R (UBX ONLY MODE)...\n")

#     # =========================
#     # 0. ปิด NMEA ทั้งหมดก่อน
#     # =========================
#     print("🛑 Disabling NMEA messages...\n")

#     nmea_msgs = [
#         (0xF0, 0x00), # GGA
#         (0xF0, 0x01), # GLL
#         (0xF0, 0x02), # GSA
#         (0xF0, 0x03), # GSV
#         (0xF0, 0x04), # RMC
#         (0xF0, 0x05), # VTG
#         (0xF0, 0x06), # GRS
#         (0xF0, 0x07), # GST
#         (0xF0, 0x08), # ZDA
#     ]

#     for cls, mid in nmea_msgs:
#         send(ser, UBXMessage('CFG','CFG-MSG', SET,
#             msgClass=cls,
#             msgID=mid,
#             rateUART1=0
#         ))

#     # =========================
#     # 1. GNSS (ลด noise)
#     # =========================
#     print("\n🛰️ Config GNSS...\n")

#     gnss = UBXMessage('CFG','CFG-GNSS', SET,
#         msgVer=0, numTrkChHw=0, numTrkChUse=0, numConfigBlocks=3,

#         # GPS
#         gnssId_01=0, resTrkCh_01=8, maxTrkCh_01=16, flags_01=0x01000001,

#         # GLONASS
#         gnssId_02=6, resTrkCh_02=4, maxTrkCh_02=12, flags_02=0x01000001,

#         # Galileo
#         gnssId_03=2, resTrkCh_03=4, maxTrkCh_03=8, flags_03=0x01000001,
#     )
#     send(ser, gnss)

#     # =========================
#     # 2. Navigation Model
#     # =========================
#     nav5 = UBXMessage('CFG','CFG-NAV5', SET,
#         mask=0x0005,
#         dynModel=4,
#         minElev=20,
#     )
#     send(ser, nav5)

#     # =========================
#     # 3. Output Rate (5Hz)
#     # =========================
#     rate = UBXMessage('CFG','CFG-RATE', SET,
#         measRate=200,
#         navRate=1,
#         timeRef=0
#     )
#     send(ser, rate)

#     # =========================
#     # 4. เปิด UBX Messages
#     # =========================
#     print("\n📡 Enabling UBX messages...\n")

#     # NAV-PVT
#     send(ser, UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x07, rateUART1=1))

#     # NAV-SAT
#     send(ser, UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x35, rateUART1=1))

#     # NAV-VELNED
#     send(ser, UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x12, rateUART1=1))

#     # NAV-HPPOSLLH
#     send(ser, UBXMessage('CFG','CFG-MSG', SET,
#         msgClass=0x01, msgID=0x14, rateUART1=1))

#     # =========================
#     # 5. Save config
#     # =========================
#     print("\n💾 Saving config...\n")

#     save = UBXMessage('CFG','CFG-CFG', SET,
#         clearMask=b'\x00\x00\x00\x00',
#         saveMask=b'\x1F\x1F\x00\x00',
#         loadMask=b'\x00\x00\x00\x00',
#         deviceMask=b'\x07\x00\x00\x00'
#     )
#     send(ser, save)

#     # =========================
#     # 6. Reset
#     # =========================
#     print("\n🔄 Resetting device...\n")

#     reset = UBXMessage('CFG','CFG-RST', SET,
#         navBbrMask=0xFFFF,
#         resetMode=0x01
#     )
#     send(ser, reset)

#     print("🎉 DONE!")
#     print("👉 ตอนนี้ device จะไม่มี NMEA แล้ว (UBX ล้วน)")



#         resetMode=0x01
#     )
#     send(ser, reset)

#     print("🎉 DONE!")
#     print("👉 ตอนนี้ device จะไม่มี NMEA แล้ว (UBX ล้วน)")


