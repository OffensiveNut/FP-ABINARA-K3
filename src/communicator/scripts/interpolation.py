import math
def interpolatesg(x):
    # Ensure angle is within range in radians
    if x < -math.pi / 2:
        x = -math.pi / 2
    elif x > math.pi / 2:
        x = math.pi / 2
    # Angle range [a, b]
    a = -math.pi / 2  # -90 degrees in radians
    b = math.pi / 2   # 90 degrees in radians
    # Value range [c, d]
    c = 1000
    d = 5000
    # Linear interpolation
    y = ((x - a) * (d - c) / (b - a)) + c
    return int(y)

def interpolatemg(x):
    # Ensure angle is within range in radians
    if x < -math.pi / 3:
        x = -math.pi / 3
    elif x > math.pi / 3:
        x = math.pi / 3
    # Angle range [a, b]
    a = -math.pi / 3  # -60 degrees in radians
    b = math.pi / 3   # 60 degrees in radians
    # Value range [c, d]
    c = 1000
    d = 5000
    # Linear interpolation
    y = ((x - a) * (d - c) / (b - a)) + c
    return int(y)
# def main():
#     # # Sudut dalam derajat
#     # angles = [100.0, -90.0, 73.34]
#     # # Nilai digital dari sudut
#     # digital_units = [interpolate(angle) for angle in angles]
#     # # Tampilkan nilai sebelum dan setelah interpolasi
#     # print(f'Sudut: {angles}')
#     # print(f'Nilai: {digital_units}')
# if __name__ == '__main__':
#     main()