def interpolate(x):
    # Pastikan sudut berada di dalam range
    if x < -90.0: x = -90.0
    elif x > 90.0: x = 90.0
    # Angle range [a, b]
    a = -90
    b = 90
    # Value range [c, d]
    c = 1000
    d = 5000
    # Persamaan interpolasi linear dari range
    y = ((x - a) * (d - c) / (b - a)) + c
    # Kembalikan nilai sebagai integer
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