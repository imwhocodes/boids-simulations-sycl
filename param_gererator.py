
import math
import time
import json
import socket


# from json import encoder
# encoder.FLOAT_REPR = lambda o: format(o, '.7f')

import statistics
# import mido

TARGET = ('localhost', 55555)

GLOBAL_ACCEL = 0.03
GLOBAL_SPEED = 0.17

MIN_DIST_A =  300
RANGE_A =     1500


MIN_DIST_B =  1700
RANGE_B =     3000

AVG_SAMPLE = 50
last_measures_A = []
last_measures_B = []


def lerpV(xv, yv, a):
    return [ ( x + ( (y-x) * a)  ) for x, y in zip( xv, yv ) ]

if __name__ == "__main__":

    last_midi_value = dict()

    for i in range(1, 9):
        last_midi_value[i] = 0


    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:

        with open('/dev/ttyACM0', 'r') as f:

            for line in f:

                params = dict()

                print(line)

                raw_val_A, raw_val_B = line.split(';')

                now_val_A = 1 - min(float(raw_val_A) - MIN_DIST_A, RANGE_A) / (RANGE_A)
                now_val_B = 1 - min(float(raw_val_B) - MIN_DIST_B, RANGE_B) / (RANGE_B)



                last_measures_A.insert(0, now_val_A)
                last_measures_B.insert(0, now_val_B)


                if len(last_measures_A) > AVG_SAMPLE:
                    last_measures_A.pop()
                    last_measures_B.pop()


                val_A = statistics.mean(last_measures_A) #mean(last_measures)
                val_B = statistics.mean(last_measures_B) #mean(last_measures)


                print(now_val_A, '\t->\t', val_A)
                print(now_val_B, '\t->\t', val_B)
                print()



                params["0"] = {

                            "size"      :   lerpV( [2, 2, 2], [7, 7, 7], now_val_A ),
                            "radiuses"  :   lerpV( [0.20, 0.35, 0.55], [0.75, 1.5, 1.5], val_B ),
                            "weights"   :   lerpV( [1, 1, 1], [1.2, 0.5, 1.2], last_midi_value[3]),
                            "angles"    :   lerpV( [720, 120, 180], [720, 90, 100], last_midi_value[4]),
                            "max_speed" :   GLOBAL_SPEED * 0.9 * (last_midi_value[5] + 0.5),
                            "accel"     :   GLOBAL_ACCEL * (last_midi_value[6] + 0.5),
                            "repel"     :   0.15
                    }


                params["1"] = {

                        "size"      :   lerpV( [2, 2, 2], [7, 7, 7], now_val_A ),
                        "radiuses"  :   lerpV( [0.50, 0.35, 0.7], [0.9, 0.80, 1], val_B ),
                        "weights"   :   lerpV( [0.6, 0.7, 0.8], [1, 1, 1], last_midi_value[3]),
                        "angles"    :   lerpV( [720, 120, 180], [720, 90, 100], last_midi_value[4]),
                        "max_speed" :   GLOBAL_SPEED * (last_midi_value[5] + 0.5),
                        "accel"     :   GLOBAL_ACCEL * 0.8 * (last_midi_value[6] + 0.5),
                        "repel"     :   0.15
                }


                params["2"] = {

                        "size"      :   lerpV( [2, 2, 2], [7, 7, 7], now_val_A ),
                        "radiuses"  :   lerpV( [0.20, 0.35, 0.55], [0.75, 1.5, 1.5], val_B ),
                        "weights"   :   lerpV( [0.5, 0.5, 0.5], [0.6, 0.5, 0.7], last_midi_value[3]),
                        "angles"    :   lerpV( [720, 120, 180], [720, 90, 100], last_midi_value[4]),
                        "max_speed" :   GLOBAL_SPEED * 1.1  * (last_midi_value[5] + 0.5),
                        "accel"     :   GLOBAL_ACCEL * 0.5 * (last_midi_value[6] + 0.5),
                        "repel"     :   0.15
                }


                params["3"] = {

                        "size"      :   lerpV( [2, 2, 2], [7, 7, 7], now_val_A ),
                        "radiuses"  :   lerpV( [0.10, 0.15, 0.20], [1.5, 2, 3], val_B ),
                        "weights"   :   lerpV( [1, 1, 1], [2, 1, 1], last_midi_value[3]),
                        "angles"    :   lerpV( [720, 120, 180], [720, 90, 100], last_midi_value[4]),
                        "max_speed" :   GLOBAL_SPEED * 1.5 * (last_midi_value[5] + 0.5),
                        "accel"     :   GLOBAL_ACCEL * 0.7 * (last_midi_value[6] + 0.5),
                        "repel"     :   0.15
                }


                j_cmd = json.dumps(params) + '\n'

                # print(j_cmd, flush=True)

                s.sendto(j_cmd.encode(), TARGET)
