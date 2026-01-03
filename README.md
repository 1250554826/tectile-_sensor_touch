#Tactile

#### Project Website(项目网页): https://github.com/1250554826/SynapTex


## 1. Firmware

(1) Load the [arduino code](/arduino_code) to the arduino. 

      MatrixArray_naive 为初始版本的arduino代码，无标定；
      MatrixArray_normal 为正常标定版本的arduino代码，适合普适范围（可感知的受压面大，但串扰偏大一些）；
      MatrixArray_update 为更新标定版本的arduino代码，适合精细范围（仅限于感知受压面小的物体，串扰小），同时MuJoCo仿真效果好

## 2. Python

(1) Setup environment

        conda create --name SynapTex python=3.10
        conda activate SynapTex
        
        pip install pyserial
        pip install opencv-python==4.6.0.66
        pip install scipy
        pip install numpy==1.23.0
        pip install mujoco==3.3.0


(2) Start python visualization(Test Real)

注意：运行触觉布料的可视化测试脚本需要将【布料硬件】连接上位机（电脑）。建议烧入MatrixArray_normal代码。

        cd python/real
        python3 multi_thread_contact_v0.py

声明：关于标准版的布料，应该运行``multi_thread_contact_new_v0.py``， 这是由于代码中应遵循以下代码逻辑：

                if current is not None and len(current) == 16:  # 如果当前帧完整
                    current_array = np.array(current)  # 将当前帧转为numpy数组
                    temp = 0
                    # 重新排列行数据，调整帧的顺序
                    # 标准版
                    new[:15, :] = current_array[:15, :] + temp  # 前15行保持不变
                        
                    # 定制版
                    # new[8, :] = current_array[15, :] + temp
                    # new[9, :] = current_array[14, :] + temp
                    # new[10, :] = current_array[13, :] + temp
                    # new[11, :] = current_array[12, :] + temp
                    # new[12, :] = current_array[11, :] + temp
                    # new[13, :] = current_array[10, :] + temp
                    # new[14, :] = current_array[9, :] + temp
                    # new[15, :] = current_array[8, :] + temp

为了上位机触觉感知可视化图像效果正常，对于标准版布料，其余的运行代码都要使用该逻辑，可以自行注释更改代码；而应用换行代码的是定制版布料。



(3)【额外补充】基于MuJoCo仿真的触觉传感器测试(Test Sim)

        cd python/sim
        python3 sim_touch_vis.py

运行``sim_touch_vis.py``代码的测试效果如下：
![img1](G:\迅雷下载\STF_touch_visualization-master(2)\STF_touch_visualization-master\image\img1.png)

## 3. Error Collection

注意： 

1) 串口端口的设置（``Linux``系统和``Windows``系统设置不同），例如
        
        Linux系统: '/dev/ttyUSB1'
        Windows系统: 'COM6'

2) 在MuJoCo仿真中，不同的XML文件路径要与自己的电脑一致，比如项目其中的一个XML文件路径设置为：


3) 在``multi_thread_contact_v0.py``代码中，阈值和噪声缩放因子可以自行调试，更改不同的数值来调整可视化中的噪声串扰：

        # 定义阈值和噪声缩放因子
        THRESHOLD = 6
        NOISE_SCALE = 20

``Reference``: https://binghao-huang.github.io/3D-ViTac/

## Citation
If you use this project in your work, please cite the following:

      @misc{jnu2025SynapTex,
      title={SynapTex},
      author={Ma, Jun and Yang},
      year={2025},
      howpublished={\url{https://github.com/1250554826/SynapTex}},
}
}
