__author__ = "Jona Gladines <<jona.gladines@uantwerpen.be>>"
__copyright__ = "Copyright (C) 2024 Jona Gladines"
__license__ = "Public Domain"
__version__ = "0.1"


import pickle
import serial
import time
import numpy as np
import logging


logging.basicConfig(filename="AR4.log",
                    filemode='a',
                    format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.DEBUG)


class AR4(object):

    def __init__(self, port):
        try:
            self.calibration = {}
            self.WC = ''
            self.loop_mode = ''
            self.spline_active = False
            self.e_stop_active = False
            self.port = port
            self.ser = None
            self.ser2 = None
            self.calibrated = False

        except Exception as e:
            print("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER - see log for details")
            logging.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
            logging.error(str(e))
            raise

    def __del__(self):
        self.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self):
        self.close()

    def open(self):
        self.ser = serial.Serial(self.port, baudrate=9600)
        print("SYSTEM READY")
        logging.info("SYSTEM READY")
        time.sleep(.1)
        self.ser.reset_input_buffer()
        self.startup()

    def close(self):
        try:
            # command = "CL"
            # self.ser.write(command.encode())
            self.ser.close()
            if self.ser2 is not None:
                self.ser2.close()
        except Exception as e:
            print(str(e))

    def startup(self):
        self.load_calibration()
        self.calc_loop_mode()
        self.update_params()
        time.sleep(.1)
        # self.cal_ext_axis()
        # time.sleep(.1)
        self.send_pos()
        time.sleep(.1)
        self.request_pos()

    def send_command(self, command):
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        if response[:1] == 'E':
            self.error_handler(response)
        else:
            self.parse_response(response)

    def parse_response(self, response):
        j1_ang_index = response.find('A')
        j2_ang_index = response.find('B')
        j3_ang_index = response.find('C')
        j4_ang_index = response.find('D')
        j5_ang_index = response.find('E')
        j6_ang_index = response.find('F')
        x_pos_index = response.find('G')
        y_pos_index = response.find('H')
        z_pos_index = response.find('I')
        rz_pos_index = response.find('J')
        ry_pos_index = response.find('K')
        rx_pos_index = response.find('L')
        speed_vio_index = response.find('M')
        debug_index = response.find('N')
        flag_index = response.find('O')
        j7_pos_index = response.find('P')
        j8_pos_index = response.find('Q')
        j9_pos_index = response.find('R')

        self.calibration['J1AngCur'] = response[j1_ang_index + 1:j2_ang_index].strip()
        self.calibration['J2AngCur'] = response[j2_ang_index + 1:j3_ang_index].strip()
        self.calibration['J3AngCur'] = response[j3_ang_index + 1:j4_ang_index].strip()
        self.calibration['J4AngCur'] = response[j4_ang_index + 1:j5_ang_index].strip()
        self.calibration['J5AngCur'] = response[j5_ang_index + 1:j6_ang_index].strip()
        self.calibration['J6AngCur'] = response[j6_ang_index + 1:x_pos_index].strip()

        if float(self.calibration['J5AngCur']) > 0:
            self.WC = "F"
        else:
            self.WC = "N"

        self.calibration['XcurPos'] = response[x_pos_index + 1:y_pos_index].strip()
        self.calibration['YcurPos'] = response[y_pos_index + 1:z_pos_index].strip()
        self.calibration['ZcurPos'] = response[z_pos_index + 1:rz_pos_index].strip()
        self.calibration['RzcurPos'] = response[rz_pos_index + 1:ry_pos_index].strip()
        self.calibration['RycurPos'] = response[ry_pos_index + 1:rx_pos_index].strip()
        self.calibration['RxcurPos'] = response[rx_pos_index + 1:speed_vio_index].strip()
        speed_violation = response[speed_vio_index + 1:debug_index].strip()
        # debug = response[DebugIndex + 1:FlagIndex].strip()
        flag = response[flag_index + 1:j7_pos_index].strip()
        self.calibration['J7PosCur'] = float(response[j7_pos_index + 1:j8_pos_index].strip())
        self.calibration['J8PosCur'] = float(response[j8_pos_index + 1:j9_pos_index].strip())
        self.calibration['J9PosCur'] = float(response[j9_pos_index + 1:].strip())

        self.save_pos_data()
        if flag != "":
            self.error_handler(flag)
        if speed_violation == '1':
            print("Max Speed Violation - Reduce Speed Setpoint or Travel Distance")
            logging.warning("Max Speed Violation - Reduce Speed Setpoint or Travel Distance")

    def update_params(self):
        j1_enc_mult = str(float(self.calibration['J1EncCPR']) / float(self.calibration['J1DriveMS']))
        j2_enc_mult = str(float(self.calibration['J2EncCPR']) / float(self.calibration['J2DriveMS']))
        j3_enc_mult = str(float(self.calibration['J3EncCPR']) / float(self.calibration['J3DriveMS']))
        j4_enc_mult = str(float(self.calibration['J4EncCPR']) / float(self.calibration['J4DriveMS']))
        j5_enc_mult = str(float(self.calibration['J5EncCPR']) / float(self.calibration['J5DriveMS']))
        j6_enc_mult = str(float(self.calibration['J6EncCPR']) / float(self.calibration['J6DriveMS']))

        command = ("UP" + "A" + self.calibration['TFx'] + "B" + self.calibration['TFy'] + "C" + self.calibration['TFz']
                   + "D" + self.calibration['TFrz'] + "E" + self.calibration['TFry'] + "F" + self.calibration['TFrx']
                   + "G" + self.calibration['J1MotDir'] + "H" + self.calibration['J2MotDir']
                   + "I" + self.calibration['J3MotDir'] + "J" + self.calibration['J4MotDir']
                   + "K" + self.calibration['J5MotDir'] + "L" + self.calibration['J6MotDir']
                   + "M" + self.calibration['J7MotDir'] + "N" + self.calibration['J8MotDir']
                   + "O" + self.calibration['J9MotDir'] + "P" + self.calibration['J1CalDir']
                   + "Q" + self.calibration['J2CalDir'] + "R" + self.calibration['J3CalDir']
                   + "S" + self.calibration['J4CalDir'] + "T" + self.calibration['J5CalDir']
                   + "U" + self.calibration['J6CalDir'] + "V" + self.calibration['J7CalDir']
                   + "W" + self.calibration['J8CalDir'] + "X" + self.calibration['J9CalDir']
                   + "Y" + self.calibration['J1PosLim'] + "Z" + self.calibration['J1NegLim']
                   + "a" + self.calibration['J2PosLim'] + "b" + self.calibration['J2NegLim']
                   + "c" + self.calibration['J3PosLim'] + "d" + self.calibration['J3NegLim']
                   + "e" + self.calibration['J4PosLim'] + "f" + self.calibration['J4NegLim']
                   + "g" + self.calibration['J5PosLim'] + "h" + self.calibration['J5NegLim']
                   + "i" + self.calibration['J6PosLim'] + "j" + self.calibration['J6NegLim']
                   + "k" + self.calibration['J1StepDeg'] + "l" + self.calibration['J2StepDeg']
                   + "m" + self.calibration['J3StepDeg'] + "n" + self.calibration['J4StepDeg']
                   + "o" + self.calibration['J5StepDeg'] + "p" + self.calibration['J6StepDeg']
                   + "q" + j1_enc_mult + "r" + j2_enc_mult + "s" + j3_enc_mult + "t" + j4_enc_mult + "u" + j5_enc_mult
                   + "v" + j6_enc_mult + "w" + self.calibration['J1ΘDHpar'] + "x" + self.calibration['J2ΘDHpar']
                   + "y" + self.calibration['J3ΘDHpar'] + "z" + self.calibration['J4ΘDHpar']
                   + "!" + self.calibration['J5ΘDHpar'] + "@" + self.calibration['J6ΘDHpar']
                   + "#" + self.calibration['J1αDHpar'] + "$" + self.calibration['J2αDHpar']
                   + "%" + self.calibration['J3αDHpar'] + "^" + self.calibration['J4αDHpar']
                   + "&" + self.calibration['J5αDHpar'] + "*" + self.calibration['J6αDHpar']
                   + "(" + self.calibration['J1dDHpar'] + ")" + self.calibration['J2dDHpar']
                   + " + " + self.calibration['J3dDHpar'] + "=" + self.calibration['J4dDHpar']
                   + "," + self.calibration['J5dDHpar'] + "_" + self.calibration['J6dDHpar']
                   + "<" + self.calibration['J1aDHpar'] + ">" + self.calibration['J2aDHpar']
                   + "?" + self.calibration['J3aDHpar'] + "{" + self.calibration['J4aDHpar']
                   + "}" + self.calibration['J5aDHpar'] + "~" + self.calibration['J6aDHpar'] + "\n")
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        self.ser.read()

    def load_calibration(self):
        try:
            calibration = pickle.load(open("ARbot.cal", "rb"))
        except Exception as e:
            print("no calibration file found - creating empty calibration file")
            logging.info("no calibration file found - creating empty calibration file")
            logging.info(str(e))
            calibration = "0"
            pickle.dump(calibration, open("ARbot.cal", "wb"))

        self.calibration['J1AngCur'] = calibration[0]
        self.calibration['J2AngCur'] = calibration[1]
        self.calibration['J3AngCur'] = calibration[2]
        self.calibration['J4AngCur'] = calibration[3]
        self.calibration['J5AngCur'] = calibration[4]
        self.calibration['J6AngCur'] = calibration[5]
        self.calibration['XcurPos'] = calibration[6]
        self.calibration['YcurPos'] = calibration[7]
        self.calibration['ZcurPos'] = calibration[8]
        self.calibration['RxcurPos'] = calibration[9]
        self.calibration['RycurPos'] = calibration[10]
        self.calibration['RzcurPos'] = calibration[11]
        self.calibration['comPort'] = calibration[12]
        self.calibration['Prog'] = calibration[13]
        self.calibration['Servo0on'] = calibration[14]
        self.calibration['Servo0off'] = calibration[15]
        self.calibration['Servo1on'] = calibration[16]
        self.calibration['Servo1off'] = calibration[17]
        self.calibration['DO1on'] = calibration[18]
        self.calibration['DO1off'] = calibration[19]
        self.calibration['DO2on'] = calibration[20]
        self.calibration['DO2off'] = calibration[21]
        self.calibration['TFx'] = calibration[22]
        self.calibration['TFy'] = calibration[23]
        self.calibration['TFz'] = calibration[24]
        self.calibration['TFrx'] = calibration[25]
        self.calibration['TFry'] = calibration[26]
        self.calibration['TFrz'] = calibration[27]
        self.calibration['J7PosCur'] = calibration[28]
        self.calibration['J8PosCur'] = calibration[29]
        self.calibration['J9PosCur'] = calibration[30]
        self.calibration['VisFileLoc'] = calibration[31]
        self.calibration['VisProg'] = calibration[32]
        self.calibration['VisOrigXpix'] = calibration[33]
        self.calibration['VisOrigXmm'] = calibration[34]
        self.calibration['VisOrigYpix'] = calibration[35]
        self.calibration['VisOrigYmm'] = calibration[36]
        self.calibration['VisEndXpix'] = calibration[37]
        self.calibration['VisEndXmm'] = calibration[38]
        self.calibration['VisEndYpix'] = calibration[39]
        self.calibration['VisEndYmm'] = calibration[40]
        self.calibration['J1calOff'] = calibration[41]
        self.calibration['J2calOff'] = calibration[42]
        self.calibration['J3calOff'] = calibration[43]
        self.calibration['J4calOff'] = calibration[44]
        self.calibration['J5calOff'] = calibration[45]
        self.calibration['J6calOff'] = calibration[46]
        self.calibration['J1OpenLoopVal'] = calibration[47]
        self.calibration['J2OpenLoopVal'] = calibration[48]
        self.calibration['J3OpenLoopVal'] = calibration[49]
        self.calibration['J4OpenLoopVal'] = calibration[50]
        self.calibration['J5OpenLoopVal'] = calibration[51]
        self.calibration['J6OpenLoopVal'] = calibration[52]
        self.calibration['com2Port'] = calibration[53]
        self.calibration['curTheme'] = calibration[54]
        self.calibration['J1CalStatVal'] = calibration[55]
        self.calibration['J2CalStatVal'] = calibration[56]
        self.calibration['J3CalStatVal'] = calibration[57]
        self.calibration['J4CalStatVal'] = calibration[58]
        self.calibration['J5CalStatVal'] = calibration[59]
        self.calibration['J6CalStatVal'] = calibration[60]
        self.calibration['J7PosLim'] = calibration[61]
        self.calibration['J7rotation'] = calibration[62]
        self.calibration['J7steps'] = calibration[63]
        self.calibration['J7StepCur'] = calibration[64]  # is this used
        self.calibration['J1CalStatVal2'] = calibration[65]
        self.calibration['J2CalStatVal2'] = calibration[66]
        self.calibration['J3CalStatVal2'] = calibration[67]
        self.calibration['J4CalStatVal2'] = calibration[68]
        self.calibration['J5CalStatVal2'] = calibration[69]
        self.calibration['J6CalStatVal2'] = calibration[70]
        self.calibration['VisBrightVal'] = calibration[71]
        self.calibration['VisContVal'] = calibration[72]
        self.calibration['VisBacColor'] = calibration[73]
        self.calibration['VisScore'] = calibration[74]
        self.calibration['VisX1Val'] = calibration[75]
        self.calibration['VisY1Val'] = calibration[76]
        self.calibration['VisX2Val'] = calibration[77]
        self.calibration['VisY2Val'] = calibration[78]
        self.calibration['VisRobX1Val'] = calibration[79]
        self.calibration['VisRobY1Val'] = calibration[80]
        self.calibration['VisRobX2Val'] = calibration[81]
        self.calibration['VisRobY2Val'] = calibration[82]
        self.calibration['zoom'] = calibration[83]
        self.calibration['pick180Val'] = calibration[84]
        self.calibration['pickClosestVal'] = calibration[85]
        self.calibration['curCam'] = calibration[86]
        self.calibration['fullRotVal'] = calibration[87]
        self.calibration['autoBGVal'] = calibration[88]
        self.calibration['mX1val'] = calibration[89]
        self.calibration['mY1val'] = calibration[90]
        self.calibration['mX2val'] = calibration[91]
        self.calibration['mY2val'] = calibration[92]
        self.calibration['J8length'] = calibration[93]
        self.calibration['J8rotation'] = calibration[94]
        self.calibration['J8steps'] = calibration[95]
        self.calibration['J9length'] = calibration[96]
        self.calibration['J9rotation'] = calibration[97]
        self.calibration['J9steps'] = calibration[98]
        self.calibration['J7calOff'] = calibration[99]
        self.calibration['J8calOff'] = calibration[100]
        self.calibration['J9calOff'] = calibration[101]
        self.calibration['GC_ST_E1'] = calibration[102]
        self.calibration['GC_ST_E2'] = calibration[103]
        self.calibration['GC_ST_E3'] = calibration[104]
        self.calibration['GC_ST_E4'] = calibration[105]
        self.calibration['GC_ST_E5'] = calibration[106]
        self.calibration['GC_ST_E6'] = calibration[107]
        self.calibration['GC_SToff_E1'] = calibration[108]
        self.calibration['GC_SToff_E2'] = calibration[109]
        self.calibration['GC_SToff_E3'] = calibration[110]
        self.calibration['GC_SToff_E4'] = calibration[111]
        self.calibration['GC_SToff_E5'] = calibration[112]
        self.calibration['GC_SToff_E6'] = calibration[113]
        self.calibration['DisableWristRotVal'] = calibration[114]
        self.calibration['J1MotDir'] = calibration[115]
        self.calibration['J2MotDir'] = calibration[116]
        self.calibration['J3MotDir'] = calibration[117]
        self.calibration['J4MotDir'] = calibration[118]
        self.calibration['J5MotDir'] = calibration[119]
        self.calibration['J6MotDir'] = calibration[120]
        self.calibration['J7MotDir'] = calibration[121]
        self.calibration['J8MotDir'] = calibration[122]
        self.calibration['J9MotDir'] = calibration[123]
        self.calibration['J1CalDir'] = calibration[124]
        self.calibration['J2CalDir'] = calibration[125]
        self.calibration['J3CalDir'] = calibration[126]
        self.calibration['J4CalDir'] = calibration[127]
        self.calibration['J5CalDir'] = calibration[128]
        self.calibration['J6CalDir'] = calibration[129]
        self.calibration['J7CalDir'] = calibration[130]
        self.calibration['J8CalDir'] = calibration[131]
        self.calibration['J9CalDir'] = calibration[132]
        self.calibration['J1PosLim'] = calibration[133]
        self.calibration['J1NegLim'] = calibration[134]
        self.calibration['J2PosLim'] = calibration[135]
        self.calibration['J2NegLim'] = calibration[136]
        self.calibration['J3PosLim'] = calibration[137]
        self.calibration['J3NegLim'] = calibration[138]
        self.calibration['J4PosLim'] = calibration[139]
        self.calibration['J4NegLim'] = calibration[140]
        self.calibration['J5PosLim'] = calibration[141]
        self.calibration['J5NegLim'] = calibration[142]
        self.calibration['J6PosLim'] = calibration[143]
        self.calibration['J6NegLim'] = calibration[144]
        self.calibration['J1StepDeg'] = calibration[145]
        self.calibration['J2StepDeg'] = calibration[146]
        self.calibration['J3StepDeg'] = calibration[147]
        self.calibration['J4StepDeg'] = calibration[148]
        self.calibration['J5StepDeg'] = calibration[149]
        self.calibration['J6StepDeg'] = calibration[150]
        self.calibration['J1DriveMS'] = calibration[151]
        self.calibration['J2DriveMS'] = calibration[152]
        self.calibration['J3DriveMS'] = calibration[153]
        self.calibration['J4DriveMS'] = calibration[154]
        self.calibration['J5DriveMS'] = calibration[155]
        self.calibration['J6DriveMS'] = calibration[156]
        self.calibration['J1EncCPR'] = calibration[157]
        self.calibration['J2EncCPR'] = calibration[158]
        self.calibration['J3EncCPR'] = calibration[159]
        self.calibration['J4EncCPR'] = calibration[160]
        self.calibration['J5EncCPR'] = calibration[161]
        self.calibration['J6EncCPR'] = calibration[162]
        self.calibration['J1ΘDHpar'] = calibration[163]
        self.calibration['J2ΘDHpar'] = calibration[164]
        self.calibration['J3ΘDHpar'] = calibration[165]
        self.calibration['J4ΘDHpar'] = calibration[166]
        self.calibration['J5ΘDHpar'] = calibration[167]
        self.calibration['J6ΘDHpar'] = calibration[168]
        self.calibration['J1αDHpar'] = calibration[169]
        self.calibration['J2αDHpar'] = calibration[170]
        self.calibration['J3αDHpar'] = calibration[171]
        self.calibration['J4αDHpar'] = calibration[172]
        self.calibration['J5αDHpar'] = calibration[173]
        self.calibration['J6αDHpar'] = calibration[174]
        self.calibration['J1dDHpar'] = calibration[175]
        self.calibration['J2dDHpar'] = calibration[176]
        self.calibration['J3dDHpar'] = calibration[177]
        self.calibration['J4dDHpar'] = calibration[178]
        self.calibration['J5dDHpar'] = calibration[179]
        self.calibration['J6dDHpar'] = calibration[180]
        self.calibration['J1aDHpar'] = calibration[181]
        self.calibration['J2aDHpar'] = calibration[182]
        self.calibration['J3aDHpar'] = calibration[183]
        self.calibration['J4aDHpar'] = calibration[184]
        self.calibration['J5aDHpar'] = calibration[185]
        self.calibration['J6aDHpar'] = calibration[186]
        self.calibration['GC_ST_WC'] = calibration[187]

    def set_com_gripper(self, port):
        try:
            baud = 115200
            self.ser2 = serial.Serial(port, baud)
            print("COMMUNICATIONS STARTED WITH ARDUINO IO BOARD - See log for details")
            logging.info("COMMUNICATIONS STARTED WITH ARDUINO IO BOARD")
        except Exception as e:
            logging.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
            logging.error(str(e))

    # ------------------------- #
    #  Robot Position Commands  #
    # ------------------------- #
    def send_pos(self):
        command = ("SP" + "A" + str(self.calibration['J1AngCur']) + "B" + str(self.calibration['J2AngCur'])
                   + "C" + str(self.calibration['J3AngCur']) + "D" + str(self.calibration['J4AngCur'])
                   + "E" + str(self.calibration['J5AngCur']) + "F" + str(self.calibration['J6AngCur'])
                   + "G" + str(self.calibration['J7PosCur']) + "H" + str(self.calibration['J8PosCur'])
                   + "I" + str(self.calibration['J9PosCur']) + "\n")
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        response = self.ser.read()
        return response

    def correct_pos(self):
        command = "CP\n"
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        self.parse_response(response)
        return response

    def request_pos(self):
        command = "RP\n"
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        self.parse_response(response)

        return (float(self.calibration['XcurPos']), float(self.calibration['YcurPos']),
                float(self.calibration['ZcurPos']), float(self.calibration['RxcurPos']),
                float(self.calibration['RycurPos']), float(self.calibration['RzcurPos']),
                float(self.calibration['J7PosCur']), float(self.calibration['J8PosCur']),
                float(self.calibration['J9PosCur']), self.WC)

    # ---------------------------- #
    #  Robot Calibration Commands  #
    # ---------------------------- #
    def cal_robot_all(self):
        # ---- STAGE 1 ---- #
        command = ("LL" + "A" + str(self.calibration['J1CalStatVal']) + "B" + str(self.calibration['J2CalStatVal'])
                   + "C" + str(self.calibration['J3CalStatVal']) + "D" + str(self.calibration['J4CalStatVal'])
                   + "E" + str(self.calibration['J5CalStatVal']) + "F" + str(self.calibration['J6CalStatVal'])
                   + "G0H0I0" + "J" + str(self.calibration['J1calOff']) + "K" + str(self.calibration['J2calOff'])
                   + "L" + str(self.calibration['J3calOff']) + "M" + str(self.calibration['J4calOff'])
                   + "N" + str(self.calibration['J5calOff']) + "O" + str(self.calibration['J6calOff'])
                   + "P" + str(self.calibration['J7calOff']) + "Q" + str(self.calibration['J8calOff'])
                   + "R" + str(self.calibration['J9calOff']) + "\n")
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        response = str(self.ser.readline().strip(), 'utf-8')
        if response[:1] == 'A':
            self.parse_response(response)
            print("Auto Calibration Stage 1 Successful")
            logging.info("Auto Calibration Stage 1 Successful")
        else:
            print("Auto Calibration Stage 1 Failed - see log for details")
            logging.error("Auto Calibration Stage 1 Failed")
            logging.error(response)
            self.error_handler(response)

        # ---- STAGE 2 ---- #
        cal_stat_val2 = (int(self.calibration['J1CalStatVal2']) + int(self.calibration['J2CalStatVal2'])
                         + int(self.calibration['J3CalStatVal2']) + int(self.calibration['J4CalStatVal2'])
                         + int(self.calibration['J5CalStatVal2']) + int(self.calibration['J6CalStatVal2']))
        if cal_stat_val2 > 0:
            command = ("LL" + "A" + str(self.calibration['J1CalStatVal2'])
                       + "B" + str(self.calibration['J2CalStatVal2']) + "C" + str(self.calibration['J3CalStatVal2'])
                       + "D" + str(self.calibration['J4CalStatVal2']) + "E" + str(self.calibration['J5CalStatVal2'])
                       + "F" + str(self.calibration['J6CalStatVal2']) + "G0H0I0"
                       + "J" + str(self.calibration['J1calOff']) + "K" + str(self.calibration['J2calOff'])
                       + "L" + str(self.calibration['J3calOff']) + "M" + str(self.calibration['J4calOff'])
                       + "N" + str(self.calibration['J5calOff']) + "O" + str(self.calibration['J6calOff'])
                       + "P" + str(self.calibration['J7calOff']) + "Q" + str(self.calibration['J8calOff'])
                       + "R" + str(self.calibration['J9calOff']) + "\n")
            self.ser.write(command.encode())
            self.ser.reset_input_buffer()
            response = str(self.ser.readline().strip(), 'utf-8')
            if response[:1] == 'A':
                self.parse_response(response)
                print("Auto Calibration Stage 2 Successful")
                logging.info("Auto Calibration Stage 2 Successful")
                self.calibrated = True
            else:
                print("Auto Calibration Stage 2 Failed - see log for details")
                logging.error("Auto Calibration Stage 2 Failed")
                logging.error(response)
                self.error_handler(response)  # todo

    def cal_robot_joint(self, joint: int):
        try:
            if not isinstance(joint, int) or joint < 1 or joint > 9:
                raise ValueError()
            joints_to_cal = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            joints_to_cal[joint - 1] = 1
            command = ("LL" + "A" + str(joints_to_cal[0]) + "B" + str(joints_to_cal[1])
                       + "C" + str(joints_to_cal[2]) + "D" + str(joints_to_cal[3]) + "E" + str(joints_to_cal[4])
                       + "F" + str(joints_to_cal[5]) + "G" + str(joints_to_cal[6]) + "H" + str(joints_to_cal[7])
                       + "I" + str(joints_to_cal[8]) + "J" + str(self.calibration['J1calOff'])
                       + "K" + str(self.calibration['J2calOff']) + "L" + str(self.calibration['J3calOff'])
                       + "M" + str(self.calibration['J4calOff']) + "N" + str(self.calibration['J5calOff'])
                       + "O" + str(self.calibration['J6calOff']) + "P" + str(self.calibration['J7calOff'])
                       + "Q" + str(self.calibration['J8calOff']) + "R" + str(self.calibration['J9calOff']) + "\n")
            self.ser.write(command.encode())
            self.ser.reset_input_buffer()
            response = str(self.ser.readline().strip(), 'utf-8')
            if response[:1] == 'A':
                self.parse_response(response)  # todo
                print("J" + str(joint) + " Calibrated Successfully")
                logging.info("J" + str(joint) + " Calibrated Successfully")
            else:
                print("J" + str(joint) + " Calibrated Failed - See log for details")
                logging.error("J" + str(joint) + " Calibrated Failed")
                logging.error(response)
                self.error_handler(response)
        except ValueError:
            print("Invalid joint number")
        except Exception as e:
            print("Unknown Exception" + str(e))

    # ----------------------- #
    #  Robot Move Commands    #
    # ----------------------- #
    # Joint move, move robot in Cartesian space using all joints (not linear)
    def move_j(self, x, y, z, rx, ry, rz, j7=0.0, j8=0.0, j9=0.0,
               spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F'):

        command = ("MJX{:.3f}Y{:.3f}Z{:.3f}Rz{:.3f}Ry{:.3f}Rx{:.3f}".format(x, y, z, rz, ry, rx)
                   + "J7{:.3f}J8{:.3f}J9{:.3f}".format(j7, j8, j9)
                   + spd_prefix + str(speed) + "Ac" + str(acceleration) + "Dc" + str(deceleration)
                   + "Rm" + str(acc_ramp) + "W" + wrist_config + "Lm" + self.loop_mode + "\n")

        self.send_command(command)

    # linear move, move robot in Cartesian space with a linear move
    def move_l(self, x, y, z, rx, ry, rz, j7=0.0, j8=0.0, j9=0.0,
               spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100,
               rnd=0, wrist_config='F', dis_wrist=False):

        if np.sign(rz) != np.sign(float(self.calibration['RzcurPos'])):
            rz = rz * -1

        command = ("MLX{:.3f}Y{:.3f}Z{:.3f}Rz{:.3f}Ry{:.3f}Rx{:.3f}".format(x, y, z, rz, ry, rx)
                   + "J7{:.3f}J8{:.3f}J9{:.3f}".format(j7, j8, j9)
                   + spd_prefix + str(speed) + "Ac" + str(acceleration) + "Dc" + str(deceleration) + "Rm"
                   + str(acc_ramp) + "Rnd" + str(rnd) + "W" + wrist_config + "Lm" + self.loop_mode
                   + "Q" + str(int(dis_wrist is True)) + "\n")

        self.send_command(command)

    # joint rotation move, move robot to specific joint angles
    def move_r(self, j1, j2, j3, j4, j5, j6, j7=0.0, j8=0.0, j9=0.0,
               spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F'):

        command = ("RJA{:.3f}B{:.3f}C{:.3f}D{:.3f}E{:.3f}F{:.3f}".format(j1, j2, j3, j4, j5, j6)
                   + "J7{:.3f}J8{:.3f}J9{:.3f}".format(j7, j8, j9) + spd_prefix + str(speed)
                   + "Ac" + str(acceleration) + "Dc" + str(deceleration) + "Rm" + str(acc_ramp)
                   + "W" + wrist_config + "Lm" + self.loop_mode + "\n")

        self.send_command(command)

    # circle move, move robot in circle, with center, start point and second point on circle for plain
    def move_c(self, x_center, y_center, z_center, rx, ry, rz,
               x_start, y_start, z_start, x_plain, y_plain, z_plain, tr_val,
               spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F'):
        # move j to the beginning (second or midpoint is start of circle)
        command = ("MJ" + "X{:.3f}Y{:.3f}Z{:.3f}".format(x_start, y_start, z_start)
                   + "Rz{:.3f}Ry{:.3f}Rx{:.3f}".format(rz, ry, rx)
                   + "Tr{:.3f}".format(tr_val) + spd_prefix + str(speed) + "Ac" + str(acceleration)
                   + "Dc" + str(deceleration) + "Rm" + str(
                    acc_ramp) + "W" + wrist_config + "Lm" + self.loop_mode + "\n")

        self.send_command(command)

        # move circle command
        command = ("MC" + "Cx{:.3f}Cy{:.3f}Cz{:.3f}".format(x_center, y_center, z_center)
                   + "Rz{:.3f}Ry{:.3f}Rx{:.3f}".format(rz, ry, rx)
                   + "Bx{:.3f}By{:.3f}Bz{:.3f}".format(x_start, y_start, z_start)
                   + "Px{:.3f}Py{:.3f}Pz{:.3f}".format(x_plain, y_plain, z_plain)
                   + "Tr{:.3f}".format(tr_val) + spd_prefix + str(speed) + "Ac" + str(acceleration)
                   + "Dc" + str(deceleration) + "Rm" + str(
                    acc_ramp) + "W" + wrist_config + "Lm" + self.loop_mode + "\n")

        self.send_command(command)

    # arc move, move robot in arc from start point over mid to end
    def move_a(self, x, y, z, rx, ry, rz, x_end, y_end, z_end, tr_val,
               spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F'):

        command = ("MA" + "X{:.3f}Y{:.3f}Z{:.3f}Rz{:.3f}Ry{:.3f}Rx{:.3f}".format(x, y, z, rz, ry, rx)
                   + "Ex{:.3f}Ey{:.3f}Ez{:.3f}Tr{:.3f}".format(x_end, y_end, z_end, tr_val) + spd_prefix + str(speed)
                   + "Ac" + str(acceleration) + "Dc" + str(deceleration) + "Rm" + str(acc_ramp) + "W" + wrist_config
                   + "Lm" + self.loop_mode + "\n")

        self.send_command(command)

    # ----------------------- #
    #  Robot IO Commands      #
    # ----------------------- #

    # enable output on arduino nano (gripper)
    def set_io_arduino(self, output, state):
        if state:
            command = "ONX" + str(output) + "\n"
        else:
            command = "OFX" + str(output) + "\n"

        self.ser2.write(command.encode())
        self.ser2.reset_input_buffer()
        time.sleep(.1)
        self.ser2.read()

    # enable output on arduino nano (gripper)
    def set_io_teensy(self, output, state):
        if state:
            command = "ONX" + str(output) + "\n"
        else:
            command = "OFX" + str(output) + "\n"

        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        self.ser.read()

    # ----------------------- #
    #  Robot Other Commands   #
    # ----------------------- #
    def start_spline(self):
        if self.spline_active:
            print("Spline already active")
            return

        self.spline_active = True

        command = "SL\n"

        self.send_command(command)

    def end_spline(self, stop_queue=False):

        if not self.spline_active:
            print("Spline not active")
            return
        self.spline_active = False

        if stop_queue:
            # self.stop()
            pass

        command = "SS\n"

        self.send_command(command)

    # test Limit switches
    def test_limit_switches(self):
        command = "TL\n"
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.05)
        response = str(self.ser.readline().strip(), 'utf-8')
        return response

    # set encoders to 1000
    def set_encoders(self):
        command = "SE\n"
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.05)
        # response = str(self.ser.readline().strip(), 'utf-8')
        self.ser.read()

    # read encoders
    def read_encoders(self):
        command = "RE\n"
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.05)
        response = str(self.ser.readline().strip(), 'utf-8')
        return response

    # Set tool center point##
    def set_tcp(self, x, y, z, rx, ry, rz):
        command = "TFA{:.3f}B{:.3f}C{:.3f}D{:.3f}E{:.3f}F{:.3f}\n".format(x, y, z, rz, ry, rx)
        self.ser.write(command.encode())
        self.ser.reset_input_buffer()
        time.sleep(.1)
        self.ser.read()

    # servo command
    def servo_cmd(self, number, position):
        command = "SV" + str(number) + "P" + str(position) + "\n"
        self.ser2.write(command.encode())
        self.ser2.reset_input_buffer()
        time.sleep(.1)
        self.ser2.read()

    def error_handler(self, response):
        # #AXIS LIMIT ERROR
        if response[1:2] == 'L':
            for i, axis in enumerate(response[2:11], start=1):
                if axis == '1':
                    message = "J" + str(i) + " Axis Limit"
                    print(message)
                    logging.error(message + ": ")
                    logging.error(response)
            # stopProg()
        # #COLLISION ERROR
        elif response[1:2] == 'C':
            for i, axis in enumerate(response[2:8], start=1):
                if axis == '1':
                    message = "J" + str(i) + " Collision or Motor Error"
                    print(message)
                    logging.error(message + ": ")
                    logging.error(response)
                    self.correct_pos()

        # #REACH ERROR
        elif response[1:2] == 'R':
            message = "Position Out of Reach"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

        # #SPLINE ERROR
        elif response[1:2] == 'S':
            message = "Spline Can Only Have Move L Types"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

        # #GCODE ERROR
        elif response[1:2] == 'G':
            message = "Gcode file not found"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

        # #E STOP BUTTON
        elif response[1:2] == 'B':
            self.e_stop_active = True
            message = "E stop Button was Pressed"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

        # #CALIBRATION ERROR
        elif response[1:2] == 'A':
            message = "J" + response[2:3] + " CALIBRATION ERROR"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

        else:
            message = "Unknown Error"
            print(message)
            logging.error(message + ": ")
            logging.error(response)

    def save_pos_data(self):
        pickle.dump(self.calibration, open("ARbot2.cal", "wb"))

    def set_joint_open_loop(self, joint: int):
        try:
            if not isinstance(joint, int) or joint < 1 or joint > 9:
                raise ValueError()

            self.calibration['J' + str(joint) + 'OpenLoopVal'] = '1'

            self.calc_loop_mode()

        except ValueError:
            print("Invalid joint number")
        except Exception as e:
            print("Unknown Exception" + str(e))

    def set_joint_closed_loop(self, joint: int):
        try:
            if not isinstance(joint, int) or joint < 1 or joint > 9:
                raise ValueError()

            self.calibration['J' + str(joint) + 'OpenLoopVal'] = '0'

            self.calc_loop_mode()

        except ValueError:
            print("Invalid joint number")
        except Exception as e:
            print("Unknown Exception" + str(e))

    def calc_loop_mode(self):
        self.loop_mode = (str(self.calibration['J1OpenLoopVal']) + str(self.calibration['J2OpenLoopVal'])
                          + str(self.calibration['J3OpenLoopVal']) + str(self.calibration['J4OpenLoopVal'])
                          + str(self.calibration['J5OpenLoopVal']) + str(self.calibration['J6OpenLoopVal']))
