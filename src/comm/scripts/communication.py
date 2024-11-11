from interpolation import interpolate
import serial

def send_serial(data_str):
    # Sesuaikan parameter berikut
    port = 'COM9'  # Ganti dengan port yang sesuai
    baudrate = 9600
    # Buka Serial Port
    with serial.Serial(port, baudrate) as ser:
        # Kirim data
        ser.write(data_str.encode())


def main():
    while True:
        # Ambil input dari pengguna untuk sudut
        # try:
        angles_input = input("Masukkan dua sudut (dipisahkan dengan spasi): ")
        angles = list(map(float, angles_input.split()))  # Ubah input menjadi daftar float    
        # Konversi sudut ke digital
        digital_units = [interpolate(angle) for angle in angles]  # Gunakan fungsi interpolate
        # Tampilkan nilai sebelum dan setelah interpolasi
        print(f'Sudut: {angles}')
        print(f'Nilai: {digital_units}')

        formatted_units = ' '.join(f'{value:04d}' for value in digital_units)
        send_serial(formatted_units)




if __name__ == '__main__':
    main()
