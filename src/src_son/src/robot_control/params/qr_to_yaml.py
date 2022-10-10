# create a function read txt file write yaml file

from asyncore import read
import numpy as np
import yaml
import os
import sys
import json


def read_from_txt(file_name):
    with open(file_name, 'r') as f:
        data = f.readlines()
    data = [x.strip() for x in data]

    data = np.array(data)

    data = data.tolist()
    data = json.dumps(data)

    return data

# write json data to jason file


def write_json(file_name, data):
    with open(file_name, 'w') as f:
        json.dump(data, f)


if __name__ == '__main__':
    yaml_data = read_from_txt("./2022_Qr_code.txt")
    write_json("QR.json", yaml_data)
