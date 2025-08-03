# ano DT V8
import struct

ANOPTV8_FSI_SADDR=1
ANOPTV8_FSI_DADDR=2
ANOPTV8_FSI_FID=3
ANOPTV8_FSI_LEN=4
ANOPTV8_FSI_DATA=6

ANOPTV8_DID_SWJ=0xFE
ANOPTV8_DID_MY=0xDF

AnoTxBuf = bytearray()

class receive(object):
    databuf = []
    _data_len = 0
    _data_cnt = 0
    _sta = 0


Recv = receive()


def _recv_reset():
    Recv.databuf = []
    Recv._sta = 0
    Recv._data_cnt = 0
    Recv._data_len = 0

def AnoPTv8FrameAnl():
    _fid = Recv.databuf[ANOPTV8_FSI_FID]
    if _fid == 0xE0:
        if Recv.databuf[ANOPTV8_FSI_DATA] == 0:
            AnoSendDevInfo()
        elif Recv.databuf[ANOPTV8_FSI_DATA] == 1:
            AnoSendParCount()
        else:
            AnoSendCheck(Recv.databuf[ANOPTV8_FSI_FID], Recv.databuf[_data_len-2], Recv.databuf[_data_len-1])
    elif _fid == 0xC1:
        if Recv.databuf[ANOPTV8_FSI_DATA] == 0:
            AnoSendCmdCount()
        else:
            AnoSendCheck(Recv.databuf[ANOPTV8_FSI_FID], Recv.databuf[_data_len-2], Recv.databuf[_data_len-1])
    else:
        AnoSendCheck(Recv.databuf[ANOPTV8_FSI_FID], Recv.databuf[_data_len-2], Recv.databuf[_data_len-1])

def _frameCheck():
    suma = 0
    sumb = 0
    len = Recv.databuf[4] + Recv.databuf[5]*256
    if len > 0:
        for i in range(len+6):
            suma += Recv.databuf[i]
            suma = suma % 256
            sumb += suma
            sumb = sumb % 256
        if suma == Recv.databuf[len+6] and sumb == Recv.databuf[len+7]:
            return True
        else:
            return False

def AnoReceiveOneByte(data):
    if Recv._sta == 0:
        if data == 0xAB:
            Recv.databuf.append(data)
            Recv._sta = 1
        else:
            _recv_reset()
    elif Recv._sta == 1:
        # saddr
        Recv.databuf.append(data)
        Recv._sta = 2
    elif Recv._sta == 2:
        # daddr
        Recv.databuf.append(data)
        Recv._sta = 3
    elif Recv._sta == 3:
        # cmd
        Recv.databuf.append(data)
        Recv._sta = 4
    elif Recv._sta == 4:
        # len0
        Recv.databuf.append(data)
        Recv._sta = 5
    elif Recv._sta == 5:
        # LEN
        Recv.databuf.append(data)
        Recv._data_len = data*256+Recv.databuf[4]
        Recv._data_cnt = 0
        if Recv._data_len > 60:
            _recv_reset()
        else:
            Recv._sta = 6
    elif Recv._sta == 6:
        # Data
        Recv.databuf.append(data)
        Recv._data_cnt = Recv._data_cnt + 1
        if Recv._data_cnt == Recv._data_len:
            Recv._sta = 7
    elif Recv._sta == 7:
        # suma
        Recv.databuf.append(data)
        Recv._sta = 8
    elif Recv._sta == 8:
        Recv.databuf.append(data)
        if _frameCheck():
            # check ok
            print("anodtv8 check ok!")
            AnoPTv8FrameAnl()
            _recv_reset()
        else:
            _recv_reset()

def AnoRecvBuf(databuf):
    _len = len(databuf)
    for i in range(_len):
        AnoReceiveOneByte(databuf[i])

def AnoDataPackSpeed(x, y, z):
    bx = x.to_bytes(length=2, byteorder='little', signed=True)
    by = y.to_bytes(length=2, byteorder='little', signed=True)
    bz = z.to_bytes(length=2, byteorder='little', signed=True)
    UserData = bytearray([0xAB, ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0x33, 0x00, 0x00, bx[0],
                         bx[1], by[0], by[1], bz[0], bz[1], 0x00, 0x00])
    _len = len(UserData)
    UserData[4] = (_len - 8) % 256
    UserData[5] = (_len - 8) / 256
    suma = 0
    sumb = 0

    for i in range(_len-2):
        suma += UserData[i]
        suma = suma % 256
        sumb += suma
        sumb = sumb % 256
    UserData[_len-2] = suma
    UserData[_len-1] = sumb
    return UserData


def AnoDataPackPosition(_id, _x, _y, _a):
    dataBuf = struct.pack("hhH", int(_x), int(_y), int(_a))
    UserData = bytearray([0xAB, ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0x35, 0x00, 0x00,
                          _id,
                          dataBuf[0], dataBuf[1],
                          dataBuf[2], dataBuf[3],
                          dataBuf[4], dataBuf[5],
                          0x00, 0x00])
    _len = len(UserData)
    UserData[4] = (_len - 8) % 256
    UserData[5] = (_len - 8) / 256
    suma = 0
    sumb = 0

    for i in range(_len-2):
        suma += UserData[i]
        suma = suma % 256
        sumb += suma
        sumb = sumb % 256
    UserData[_len-2] = suma
    UserData[_len-1] = sumb
    return UserData

def AnoDataPackCamImg(_img,_flag,_color,_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8,_v9,_v10):
    #图像数据数组
    _imgdata = _img.bytearray();
    #图像宽度
    _width = _img.width()
    #图像高度
    _height = _img.height()
    #图像数据长度
    pixDataLen = len(_imgdata)

    bufLen = pixDataLen + 25
    dataLen = pixDataLen + 17

    dataBuf = bytearray(bufLen)
    dataBuf[0] = 0xAB
    dataBuf[1] = ANOPTV8_DID_MY
    dataBuf[2] = ANOPTV8_DID_SWJ
    dataBuf[3] = 0xB0

    _tmp = struct.pack("HHHH", int(dataLen),int(_flag), int(_width), int(_height))
    #len
    dataBuf[4] = _tmp[0]
    dataBuf[5] = _tmp[1]
    #flag
    dataBuf[6] = _tmp[2]
    dataBuf[7] = _tmp[3]
    #color
    dataBuf[8] = _color
    #width
    dataBuf[9] = _tmp[4]
    dataBuf[10] = _tmp[5]
    #height
    dataBuf[11] = _tmp[6]
    dataBuf[12] = _tmp[7]

    dataBuf[13] = int(_v1)
    dataBuf[14] = _v2
    dataBuf[15] = _v3
    dataBuf[16] = _v4
    dataBuf[17] = _v5
    dataBuf[18] = _v6
    dataBuf[19] = _v7
    dataBuf[20] = _v8
    dataBuf[21] = _v9
    dataBuf[22] = _v10

    i = 0
    while i < pixDataLen:
        dataBuf[23+i] = _imgdata[i]
        i += 1

    suma = 0
    sumb = 0
    for i in range(bufLen-2):
        suma += dataBuf[i]
        suma = suma % 256
        sumb += suma
        sumb = sumb % 256
    dataBuf[bufLen-2] = suma
    dataBuf[bufLen-1] = sumb

    return dataBuf

def AnoSendBuf(saddr, daddr, fid, databuf):
    _len = len(databuf) + 8
    _sbuf = bytearray(_len)
    _sbuf[0] = 0xAB
    _sbuf[1] = saddr
    _sbuf[2] = daddr
    _sbuf[3] = fid
    #len
    _sbuf[4] = (_len-8) % 256
    _sbuf[5] = int((_len-8) / 256)
    for i in range(_len-8):
        _sbuf[6+i] = databuf[i]

    suma = 0
    sumb = 0
    for i in range(_len-2):
        suma += _sbuf[i]
        suma = suma % 256
        sumb += suma
        sumb = sumb % 256
    _sbuf[_len-2] = suma
    _sbuf[_len-1] = sumb
    AnoTxBuf.extend(_sbuf)

def AnoSendCheck(fid, sc, ac):
    _sbuf = bytearray([fid, sc, ac])
    AnoSendBuf(ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0x00, _sbuf)

def AnoSendDevInfo():
    _sbuf = bytearray([ANOPTV8_DID_MY, 1,0, 2,0, 3,0, 4,0])
    _sbuf.extend(bytearray("AnoMV v8", 'utf-8'))
    AnoSendBuf(ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0xE3, _sbuf)

def AnoSendParCount():
    _sbuf = bytearray([0x01, 0,0])
    AnoSendBuf(ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0xE0, _sbuf)

def AnoSendCmdCount():
    _sbuf = bytearray([0x00, 0,0])
    AnoSendBuf(ANOPTV8_DID_MY, ANOPTV8_DID_SWJ, 0xC1, _sbuf)
