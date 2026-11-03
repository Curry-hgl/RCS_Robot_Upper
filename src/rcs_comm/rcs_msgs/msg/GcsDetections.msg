# 一帧物块或黑环结果；通信最多发送前三个，空列表表示本帧未检测到。
uint8 BLOCK=0
uint8 RING=1
std_msgs/Header header
uint8 mode
GcsDetection[] objects
