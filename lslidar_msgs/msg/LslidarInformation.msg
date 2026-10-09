string lidar_ip                 # 雷达ip地址
string destination_ip           # 雷达目的ip地址
string lidar_mac_address        # 雷达 MAC 地址
uint16 msop_port                # 雷达目的数据端口
uint16 difop_port               # 雷达目的设备端口
string lidar_serial_number      # 雷达序列号
string fpga_board_2_program     # FPGA 2号板程序版本
string fpga_board_3_program     # FPGA 3号板程序版本

# Decoda additions, filled by the CH driver from each DIFOP packet (0 / unknown
# for other models).
uint8 DUST_MODE_UNKNOWN = 255
uint16 motor_speed_rpm          # Measured motor speed, DIFOP bytes 8-9. 300/600/1200 rpm = 5/10/20 Hz; reads near, not exactly at, the setpoint
uint8 time_service_mode         # DIFOP byte 44: 0 GPS, 1 gPTP, 2 NTP
uint8 dust_mode 255             # CH128X1 only, DIFOP byte 51 (empirical): 0 off, 1 on; DUST_MODE_UNKNOWN otherwise
