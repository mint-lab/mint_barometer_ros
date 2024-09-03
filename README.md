# magic_box_ros  
A ROS 2 package for position estimation using a portable magic box equipped with multiple sensors  

## Prerequisite  
**Install [i2c-ch341-usb](https://github.com/gschorcht/i2c-ch341-usb)  kernel driver**  
- If you are using Ubuntu 22.04 LTS, you need to modify `i2c-ch341-usb.c` as follows due to kernel compatibility issues

![file_modify](https://github.com/user-attachments/assets/9a3c5d6f-03cc-4970-80a5-82a27cebadbf)
