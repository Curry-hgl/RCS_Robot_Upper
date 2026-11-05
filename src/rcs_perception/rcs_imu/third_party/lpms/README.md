LP-Research LPMS SDK，来源：resource/ALB_IMU 中已验证的 BE ROS2 例程配套核心库。
保留原始文件名及版权声明；许可见 LICENSE.txt。
本地修正：初始化线程状态、停止后回收线程对象、连接状态使用原子变量、设置读写加锁。
仅编译 Linux 源文件，静态链接到本包，不安装系统全局驱动。
