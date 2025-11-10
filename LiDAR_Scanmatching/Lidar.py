import serial
import time
import sys

DEBUG_MODE = False

"""
Urg class
"""
class Urg:
    """
    Urgクラスの定義
    """
    def __init__(self, device_file, baurate):
        self.device_file = device_file
        self.baurate = baurate
        self.timeout = 1
        try:
            self.ser = serial.Serial(self.device_file, self.baurate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Connection error {e}", file=sys.stderr)
            print("Bye")
            sys.exit(0)
        time.sleep(2)

        # VVコマンドを送信し通信テスト
        success, response = cmd_VV(self.ser)
        if success is True:
            print("[OK] VV")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] VV", file=sys.stderr)
            sys.exit(0)

        time.sleep(1)

        # PPコマンドを送信し通信テスト
        success, response = cmd_PP(self.ser)
        if success is True:
            print("[OK] PP")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] PP", file=sys.stderr)
            sys.exit(0)

        time.sleep(1)

        # IIコマンドを送信し通信テスト
        success, response = cmd_II(self.ser)
        if success is True:
            print("[OK] II")
            if DEBUG_MODE:
                print(response, file=sys.stderr)
        else:
            print("[False] II", file=sys.stderr)
            sys.exit(0)

        # 通信テストを合格
        print("Connect test all clear.")

    def one_shot(self):
        """
        1回だけ計測する
        """
        # MDコマンドを送信しone-shot計測
        success, _, data = cmd_MD(self.ser)
        urg_data = []
        if success is True:
            data_strings = ""
            for p in data:
                if len(p) > 0:
                    # check_sum = p[-1]
                    body = p[:-1]
                    data_strings += body
            for i in range(0, len(data_strings), 3):
                three_chars = data_strings[i:i+3]
                binary = []
                for char in three_chars:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit = ''.join(binary)
                if all(c in '01' for c in combined_binary_24bit):
                    r = int(combined_binary_24bit, 2)
                    index = int(i/3.0)
                    urg_data.append((index, r))
                else:
                    print("不正なデータが含まれています")
                    self.close()
                    sys.exit(0)
            return True, urg_data
        return False, urg_data

    def one_shot_intensity(self):
        """
        1回だけ計測する（反射強度付き）
        """
        # MEコマンドを送信しone-shot計測
        success, _, data = cmd_ME(self.ser)
        urg_data = []
        if success is True:
            data_strings = ""
            for p in data:
                if len(p) > 0:
                    # check_sum = p[-1]
                    body = p[:-1]
                    data_strings += body
            print(f"data_length:{len(data_strings)}")
            print(data_strings)
            print("===")
            for i in range(0, len(data_strings), 6):
                three_chars_r = data_strings[i+0:i+3]
                three_chars_i = data_strings[i+3:i+6]
                binary_r = []
                binary_i = []
                for char in three_chars_r:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary_r.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit_r = ''.join(binary_r)

                for char in three_chars_i:
                    ch = ord(char) - 0x30
                    hex_ascii_value = hex(ch)
                    bin_val = bin(int(hex_ascii_value, 16))
                    # 2進数を6ビットの2進数に変換
                    binary_i.append(format(int(bin_val, 2), '06b'))
                combined_binary_24bit_i = ''.join(binary_i)
                if all(c in '01' for c in combined_binary_24bit_r) or all(c in '01' for c in combined_binary_24bit_i):
                    r = int(combined_binary_24bit_r, 2)
                    intensity = int(combined_binary_24bit_i, 2)
                    index = int(i/3.0)
                    urg_data.append((index, r, intensity))
                else:
                    print("不正なデータが含まれています")
                    self.close()
                    sys.exit(0)
            return True, urg_data
        return False, urg_data

    def close(self):
        """
        close serial port
        """
        self.ser.close()

def remove_semicolon_followed_by_char(line):
    # 改行を除去する
    line = line.rstrip('\n')
    # 「;」の位置を見つける
    semicolon_index = line.find(';')
    if semicolon_index != -1:
        # 「;」の前の部分のみを抽出
        return line[:semicolon_index]
    return line

def cmd_VV(ser_dev):
    # VVコマンドを送信（デバイス情報を要求）
    ser_dev.write(b'VV\n')
    # 応答を読み取る
    ret = []
    for _ in range(8):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret

    return False, ret

def cmd_PP(ser_dev):
    # PPコマンドを送信（デバイスパラメータ情報を要求）
    ser_dev.write(b'PP\n')
    # 応答を読み取る
    ret = []
    for _ in range(11):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret
    return False, ret

def cmd_II(ser_dev):
    # IIコマンドを送信（ステータス情報を要求）
    ser_dev.write(b'II\n')
    # 応答を読み取る
    ret = []
    for _ in range(10):
        response = ser_dev.read_until().decode('utf-8')
        ret.append(remove_semicolon_followed_by_char(response))
    if len(ret) > 0:
        return True, ret
    return False, ret

def cmd_MD(ser_dev):
    # MDコマンドを送信（距離データの取得）
    ser_dev.write(b'MD0000108001101\n')
    #ser_dev.write(b'MD0044072501101\n') # for Classic URG
    # 応答を読み取る
    head = []
    data = []
    for _ in range(6):
        response = ser_dev.read_until().decode('utf-8')
        head.append(remove_semicolon_followed_by_char(response))
    LOOP_NUM_FOR_READ = 52  # Classic URGでは33
    for _ in range(LOOP_NUM_FOR_READ):
        response = ser_dev.read_until().decode('utf-8')
        response = response.rstrip('\n')
        data.append(response)
    if len(head) > 0:
        return True, head, data
    return False, head, []

def cmd_ME(ser_dev):
    # MDコマンドを送信（距離データの取得）
    ser_dev.write(b'ME0000108001101\n')
    #ser_dev.write(b'MD0044072501101\n') # for Classic URG
    # 応答を読み取る
    head = []
    data = []
    for _ in range(6):
        response = ser_dev.read_until().decode('utf-8')
        head.append(remove_semicolon_followed_by_char(response))
    LOOP_NUM_FOR_READ = 103  # Classic URGでは33
    for _ in range(LOOP_NUM_FOR_READ):
        response = ser_dev.read_until().decode('utf-8')
        response = response.rstrip('\n')
        data.append(response)
    if len(head) > 0:
        return True, head, data
    return False, head, []



def index2angle(index):
    return (index + 0) * 360/1440.0 - 135.0
    # 以下の数値はUBG-04LX-F01の設定値
    #return (index + 44) * 360/1024 - 135.0

def angle2index(angle):
    return int((angle - (-135.0)*1440.0/360))

def deg2rad(deg):
    return deg * pi/180.0

