# Dual IMU with SPI + FD-CAN

This unit runs on the [STM32G4](https://www.st.com/resource/en/datasheet/stm32g473cb.pdf) chip and has two IMUs: (1) [Murata](https://sensorsandpower.angst-pfister.com/fileadmin/products/datasheets/191/SCHA63T-K03-rev3_1640-21648-0029-E-1121.pdf) and (2) [TDK](https://invensense.tdk.com/wp-content/uploads/2021/11/DS-000409-IAM-20680HP-v1.2-Typ.pdf). STM-IMU communication is SPI and output is FD-CAN. 

## Dev flow
### First time setup
1) Install tools from STM32 [here](https://www.google.com/url?q=https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step1_Tools_installation&sa=D&source=docs&ust=1703379897873265&usg=AOvVaw3_c3vA3EHbkvutsGrSWoX8). 
    * Download STM32CubeIDE if you are configuring the chip OR changing code, e.g. modifying, debugging or deploying code to the chip. Current versions of CubeIDE have CubeMX built in so you won't need to download both. 
    * (You probably don't need this) For older builds, download STM32CubeMX if you are only configuring the chip, e.g. changing SPI, FD-CAN or pins.

2) Set up communication with serial devices over USB
    * Download [CoolTerm](https://coolterm.en.lo4d.com/windows), [PuTTY](https://pbxbook.com/voip/sputty.html), or use your [terminal](https://pbxbook.com/other/mac-tty.html). We use this to capture printf statements from the STM chip, which are piped to UART at baud rate 115200. 
    * Set up STM32CubeIDE.
        * Open up STM32CubeIDE. When prompted to "Select a directory as workspace" choose something that is not <path/to/Dual_IMU>.
        * Select File > Open Projects from File System > Directory > <path/to/Dual_IMU>. In the left sidebar titled "Project Explorer", you should now see the Dual_IMU directory. 
        * Enable printing floats. You only need to do this if generating a new makefile from .ioc. Right click the Dual_IMU directory > Properties > C/C++ Build > Settings > MCU Settings > check "Use float with printf from newlib-nano" > Apply and Close

### Make changes
1) To change the STM chip config:
    * Open `Dual_IMU/Dual_IMU.ioc` in CubeIDE. Then make changes in the GUI. 
    * When finished, click File > Save. A popup will ask "Do you want to generate code?". Click "Yes". 
2)  To change code:
    * The main code lives in [`Dual_IMU/Core/Src`](/Dual_IMU/Core/Src/). Make sure your changes happen inside sections marked `/* USER CODE BEGIN ... */` `/* USER CODE END ... */`. Code outside these markers will be overwritten by CubeIDE when generating code after changes to the `.ioc` file.
3) Build + Run:
    * Open CubeIDE GUI to build, debug, or run.