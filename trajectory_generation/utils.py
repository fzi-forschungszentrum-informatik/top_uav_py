import math


def r(number):
    try:
        out = round(number * 100000000) / 100000000
    except:
        out = number
    return out


def r3(number):
    try:
        out = round(number * 1000) / 1000
    except:
        out = number
    return out


def remove_duplicates(my_list):
    return list(dict.fromkeys(my_list))


def normalize_angles(rad):
    while rad > 2 * math.pi:
        rad -= 2 * math.pi
    return rad
