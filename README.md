# STM32F407 TFT-LCD 触控屏驱动工程

## 项目概述

本项目是基于 **STM32F407ZGT6** 微控制器和 **TFT-LCD (480×800)** 触控屏的完整驱动实现。支持电阻式和电容式触摸屏，兼容多种触摸控制器芯片（GT9147/GT9271/FT5206等）和LCD驱动IC（ILI9341/NT35310/NT35510/SSD1963等）。

### 硬件平台
- **主控芯片**: STM32F407ZGT6 (Cortex-M4, 168MHz)
- **LCD屏幕**: TFT-LCD 480×800分辨率，MCU并口驱动
- **触摸方案**: 
  - 电阻式触摸：支持 ADS7843/XPT2046/TSC2046 等
  - 电容式触摸：支持 GT9147/GT9271/FT5206 等
- **通信接口**: 
  - LCD: FSMC (灵活静态存储控制器)
  - 触摸: IIC (电容屏) / SPI (电阻屏)

---

## 工程结构

```
LcdTouchRef/
├── Core/                           # STM32 HAL核心代码
│   ├── Inc/                        # 头文件
│   │   ├── main.h                  # 主程序头文件
│   │   ├── fsmc.h                  # FSMC外设头文件
│   │   ├── gpio.h                  # GPIO配置头文件
│   │   └── usart.h                 # 串口配置头文件
│   └── Src/                        # 源文件
│       ├── main.c                  # 主程序（触摸屏测试演示）
│       ├── fsmc.c                  # FSMC初始化配置
│       ├── gpio.c                  # GPIO初始化
│       └── stm32f4xx_it.c          # 中断服务程序
│
├── BSP/                            # 板级支持包（驱动层）
│   ├── LCD/                        # LCD驱动模块
│   │   ├── lcd.c/lcd.h             # LCD核心驱动（FSMC接口）
│   │   ├── lcd_ex.c                # LCD扩展驱动（多芯片支持）
│   │   ├── lcdfont.h               # 字库数据
│   │   ├── delay.c/delay.h         # 延时函数
│   │   └── sys.c/sys.h             # 系统支持函数
│   │
│   ├── TOUCH/                      # 触摸屏驱动模块
│   │   ├── touch.c/touch.h         # 触摸屏统一接口
│   │   ├── ft5206.c/ft5206.h       # FT5206电容触摸驱动（IIC）
│   │   ├── gt9xxx.c/gt9xxx.h       # GT9xxx系列电容触摸驱动（IIC）
│   │   └── ctiic.c/ctiic.h         # 电容屏IIC通信接口
│   │
│   ├── IIC/                        # IIC总线驱动
│   │   └── myiic.c/myiic.h         # 软件模拟IIC
│   │
│   ├── 24CXX/                      # EEPROM驱动（用于触摸校准数据存储）
│   │   └── 24cxx.c/24cxx.h         # 24C系列EEPROM驱动
│   │
│   └── SYSTEM/                     # 系统级驱动
│       ├── delay/                  # 延时函数
│       └── sys/                    # 系统初始化
│
├── Drivers/                        # STM32 HAL库
│   ├── CMSIS/                      # CMSIS标准接口
│   └── STM32F4xx_HAL_Driver/       # STM32F4 HAL驱动库
│
├── MDK-ARM/                        # Keil MDK工程文件
│   ├── LcdTouch.uvprojx            # Keil工程主文件
│   └── startup_stm32f407xx.s       # 启动文件
│
├── doc/                            # 文档资料
│   └── 探索者V3 IO引脚分配表.csv   # 硬件引脚映射表
│
├── LcdTouch.ioc                    # STM32CubeMX配置文件
├── keilkilll.bat                   # Keil工程清理脚本
└── LICENSE                         # 开源协议
```

---

## 快速使用指南

### 1. 环境准备

#### 软件要求
- **Keil MDK-ARM** (推荐 V5.30 及以上版本)
- **STM32CubeMX** (可选，用于重新配置外设)
- **ST-Link** 驱动程序（用于下载和调试）

#### 硬件连接
参考 `doc/探索者V3 IO引脚分配表.csv` 确认以下连接：

**LCD 连接 (FSMC Bank4):**
- `D0-D15`: FSMC 数据总线
- `PG12 (FSMC_NE4)`: 片选信号 (CS)
- `PF12 (FSMC_A6)`: 命令/数据选择 (RS/DC)
- `PD5`: 写使能 (WR)
- `PD4`: 读使能 (RD)
- `PB15`: 背光控制 (BL)

**电容触摸屏连接 (IIC):**
- `PB6/PB7`: IIC SCL/SDA（根据具体芯片可能不同）
- `PB1`: 触摸中断引脚 (INT)
- `PC13`: 触摸复位引脚 (RST)

### 2. 编译下载

1. 打开 `MDK-ARM/LcdTouch.uvprojx` 工程文件
2. 选择目标芯片型号（已配置为 STM32F407ZGTx）
3. 点击编译按钮 (F7) 进行全编译
4. 连接 ST-Link 调试器到目标板
5. 点击下载按钮 (F8) 将程序烧录到芯片

### 3. 运行效果

程序运行后：
- LCD显示初始化信息和提示文字
- 用手指触摸屏幕任意位置
- 屏幕实时显示触摸坐标 (X, Y)
- 在触摸点绘制红色圆圈标记
- LED0 (PF9) 闪烁指示系统正常运行

### 4. 代码示例

```c
// 主循环代码片段 (main.c)
while (1)
{
    key = tp_dev.scan(0);           // 扫描触摸屏
    
    if(key)
    {
        // 显示触摸坐标
        lcd_show_xnum(10, 90, tp_dev.x[0], 4, 16, 0, BLUE);
        lcd_show_xnum(70, 90, tp_dev.y[0], 4, 16, 0, BLUE);
        
        // 在触摸点绘制圆圈
        lcd_draw_circle(tp_dev.x[0], tp_dev.y[0], 5, RED);
    }
    
    HAL_Delay(10);  // 延时10ms
}
```

---

## 二次开发指南

### 修改LCD驱动芯片
如需更换不同的LCD驱动IC，修改以下文件：
1. 在 [lcd_ex.c](BSP/LCD/lcd_ex.c) 中添加新的初始化序列
2. 在 [lcd.c](BSP/LCD/lcd.c) 的 `lcd_init()` 函数中添加ID识别逻辑

### 修改触摸屏芯片
系统自动识别触摸芯片类型（GT9xxx 或 FT5206），如需添加新芯片：
1. 参考 [ft5206.c](BSP/TOUCH/ft5206.c) 或 [gt9xxx.c](BSP/TOUCH/gt9xxx.c) 创建驱动文件
2. 在 [touch.c](BSP/TOUCH/touch.c) 的 `tp_init()` 函数中添加识别和初始化代码

### 触摸屏校准
电阻式触摸屏需要校准，校准数据存储在 24C02 EEPROM 中：
```c
tp_adjust();  // 执行触摸校准流程
```

---

## 技术原理详解

### 一、LCD驱动原理

#### 1.1 FSMC总线接口

STM32的 **FSMC (Flexible Static Memory Controller)** 是一个灵活的静态存储控制器，可以外接 SRAM、NOR Flash、LCD 等设备。本项目使用 FSMC 的 Bank1 的第4片选区 (NE4) 来驱动 LCD。

**FSMC工作原理：**

FSMC 将外部设备映射到 STM32 的内存地址空间，通过读写特定地址实现与外部设备的通信：

```
地址空间划分 (Bank1):
- FSMC_NE1: 0x60000000 - 0x63FFFFFF (64MB)
- FSMC_NE2: 0x64000000 - 0x67FFFFFF (64MB)
- FSMC_NE3: 0x68000000 - 0x6BFFFFFF (64MB)
- FSMC_NE4: 0x6C000000 - 0x6FFFFFFF (64MB) ← 本项目使用
```

**地址线复用实现命令/数据分离：**

LCD驱动需要区分"命令"和"数据"，通过 FSMC 的地址线来实现：
- 使用 `FSMC_A6` 作为 LCD 的 RS (Register Select) 引脚
- 当 A6=0 时，访问命令寄存器 (写命令)
- 当 A6=1 时，访问数据寄存器 (写数据)

地址计算公式：
```c
// 基地址 = 0x6C000000 (FSMC_NE4)
// A6 对应偏移 = (1 << (6+1)) = 0x80
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x00000080))
#define LCD_CMD         *(volatile uint16_t *)(LCD_BASE)      // A6=0, 命令
#define LCD_DATA        *(volatile uint16_t *)(LCD_BASE + 2)  // A6=1, 数据
```

实际代码实现 ([lcd.h](BSP/LCD/lcd.h#L73-L78)):
```c
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x000000FE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

typedef struct
{
    volatile uint16_t LCD_REG;  // 命令寄存器
    volatile uint16_t LCD_RAM;  // 数据寄存器
} LCD_TypeDef;
```

**FSMC时序配置：**

FSMC 需要配置读写时序以匹配 LCD 的时序要求 ([fsmc.c](Core/Src/fsmc.c)):
```c
// 写时序
Timing.AddressSetupTime = 15;       // 地址建立时间
Timing.DataSetupTime = 60;          // 数据建立时间
Timing.BusTurnAroundDuration = 0;
Timing.AccessMode = FSMC_ACCESS_MODE_A;

// 读时序
ExtTiming.AddressSetupTime = 15;
ExtTiming.DataSetupTime = 60;
ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;
```

#### 1.2 LCD初始化流程

LCD初始化分为硬件初始化和芯片初始化两部分：

**硬件初始化步骤：**
1. 配置 FSMC GPIO (数据线 D0-D15, 控制线 RS/WR/RD/CS)
2. 配置 FSMC 时序参数
3. 使能 FSMC Bank1 NE4

**芯片初始化步骤：**
1. 读取 LCD ID，识别驱动芯片型号
2. 根据不同芯片执行对应的初始化序列
3. 配置显示参数（方向、颜色模式、刷新率等）

代码示例 ([lcd.c](BSP/LCD/lcd.c)):
```c
void lcd_init(void)
{
    // 1. 读取LCD ID
    lcddev.id = lcd_read_id();
    
    // 2. 根据ID执行初始化
    if(lcddev.id == 0x9341)
    {
        lcd_ex_ili9341_reginit();  // ILI9341初始化序列
    }
    else if(lcddev.id == 0x5310)
    {
        lcd_ex_nt35310_reginit();  // NT35310初始化序列
    }
    // ... 其他芯片
    
    // 3. 设置显示方向和窗口
    lcd_display_dir(0);  // 0:竖屏, 1:横屏
    lcd_clear(WHITE);    // 清屏
}
```

#### 1.3 显示原理

**像素数据格式：**
- 使用 RGB565 格式：16位表示一个像素
  - R (5位): 红色分量 (0-31)
  - G (6位): 绿色分量 (0-63)
  - B (5位): 蓝色分量 (0-31)

```c
// 颜色定义示例
#define WHITE    0xFFFF  // 白色
#define BLACK    0x0000  // 黑色
#define RED      0xF800  // 红色 (11111 000000 00000)
#define GREEN    0x07E0  // 绿色 (00000 111111 00000)
#define BLUE     0x001F  // 蓝色 (00000 000000 11111)
```

**绘图基本流程：**
1. 设置绘图窗口 (起始坐标、结束坐标)
2. 发送写 GRAM 命令
3. 连续写入像素数据

```c
// 画点函数示例
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_set_cursor(x, y);          // 设置光标位置
    lcd_write_ram_prepare();       // 准备写GRAM
    LCD->LCD_RAM = color;          // 写入颜色数据
}
```

**坐标系统：**
- 原点 (0,0) 在左上角
- X轴向右递增，Y轴向下递增
- 支持横屏/竖屏切换（通过修改扫描方向寄存器）

---

### 二、触摸屏驱动原理

#### 2.1 电容触摸屏原理

电容式触摸屏基于电容感应原理，由触摸控制器芯片（如 GT9147、FT5206）检测触摸位置。

**工作原理：**
1. 触摸屏表面覆盖透明 ITO (氧化铟锡) 导电层
2. 手指触摸时，人体电容与屏幕形成耦合电容
3. 触摸控制器检测电容变化，计算触摸坐标
4. 通过 IIC 接口将坐标数据传输给主控

**优势：**
- 支持多点触控（本项目支持最多10点）
- 无需按压，轻触即可响应
- 高透光率，显示效果好
- 长寿命，不易磨损

#### 2.2 IIC通信协议

电容触摸芯片使用 **IIC (I2C)** 总线进行通信：

**IIC总线特性：**
- 两线制：SCL (时钟线)、SDA (数据线)
- 主从模式：STM32 为主机，触摸芯片为从机
- 半双工通信
- 标准速率 100kbps，快速模式 400kbps

**软件模拟IIC实现 ([ctiic.c](BSP/TOUCH/ctiic.c))：**
```c
// IIC起始信号
void ct_iic_start(void)
{
    CT_SDA(1);  // SDA高
    CT_SCL(1);  // SCL高
    delay_us(4);
    CT_SDA(0);  // SDA拉低（在SCL高电平期间）
    delay_us(4);
    CT_SCL(0);  // 钳住总线，准备发送数据
}

// IIC停止信号
void ct_iic_stop(void)
{
    CT_SCL(0);
    CT_SDA(0);  // SDA低
    delay_us(4);
    CT_SCL(1);  // SCL高
    CT_SDA(1);  // SDA拉高（在SCL高电平期间）
    delay_us(4);
}

// 发送一个字节
void ct_iic_send_byte(uint8_t data)
{
    for(int i = 0; i < 8; i++)
    {
        CT_SDA((data & 0x80) >> 7);  // 从高位开始发送
        data <<= 1;
        delay_us(2);
        CT_SCL(1);  // 上升沿，从机读取数据
        delay_us(2);
        CT_SCL(0);
    }
}
```

#### 2.3 触摸数据读取流程

**GT9xxx 系列触摸芯片数据读取：**

1. **读取触摸状态寄存器 (0x814E)：**
```c
gt9xxx_rd_reg(GT9XXX_GSTID_REG, &mode, 1);
```
- 低4位：当前触摸点数量 (0-10)
- 高4位：触摸标志和其他状态

2. **读取触摸点坐标数据：**
每个触摸点占用 8 字节数据：
```
偏移   | 字节数 | 说明
-------|--------|------------------
0x00   | 1      | Track ID (触摸点ID)
0x01   | 2      | X坐标低字节、高字节
0x03   | 2      | Y坐标低字节、高字节
0x05   | 2      | 触摸面积
0x07   | 1      | 保留
```

代码实现 ([gt9xxx.c](BSP/TOUCH/gt9xxx.c)):
```c
uint8_t gt9xxx_scan(uint8_t mode)
{
    uint8_t buf[4];
    uint8_t res = 0;
    uint8_t temp;
    static uint8_t t = 0;
    
    // 读取触摸状态
    gt9xxx_rd_reg(GT9XXX_GSTID_REG, &mode, 1);
    
    if(mode & 0x80 && ((mode & 0x0F) <= 10))  // 有触摸且点数有效
    {
        temp = mode & 0x0F;  // 触摸点数
        
        // 读取第一个触摸点数据
        gt9xxx_rd_reg(GT9XXX_TP1_REG, buf, 4);
        
        // 解析坐标（小端模式）
        tp_dev.x[0] = ((uint16_t)buf[1] << 8) + buf[0];
        tp_dev.y[0] = ((uint16_t)buf[3] << 8) + buf[2];
        
        // 坐标转换（根据LCD方向调整）
        if(tp_dev.touchtype & 0x01)
        {
            tp_dev.x[0] = lcddev.width - tp_dev.x[0];
            tp_dev.y[0] = lcddev.height - tp_dev.y[0];
        }
        
        // 清除触摸状态标志
        temp = 0;
        gt9xxx_wr_reg(GT9XXX_GSTID_REG, &temp, 1);
        
        res = 1;  // 有触摸
    }
    
    return res;
}
```

**FT5206 系列触摸芯片类似流程：**
- 读取寄存器 0x02 获取触摸点数
- 从寄存器 0x03 开始读取各触摸点坐标（每点6字节）

#### 2.4 触摸屏初始化与识别

系统启动时自动检测触摸芯片类型：

```c
uint8_t tp_init(void)
{
    // 1. 配置GPIO
    ct_iic_init();  // IIC引脚初始化
    
    // 2. 尝试初始化GT9xxx
    if(gt9xxx_init() == 0)
    {
        tp_dev.scan = gt9xxx_scan;  // 绑定扫描函数
        tp_dev.touchtype |= 0x80;   // 标记为电容屏
        return 0;
    }
    
    // 3. 尝试初始化FT5206
    if(ft5206_init() == 0)
    {
        tp_dev.scan = ft5206_scan;
        tp_dev.touchtype |= 0x80;
        return 0;
    }
    
    // 4. 如果都失败，尝试电阻屏初始化
    // ...
    
    return 1;  // 初始化失败
}
```

#### 2.5 触摸坐标校准（电阻屏）

电阻式触摸屏由于机械压力感应，需要进行校准才能准确对应LCD坐标。

**校准原理：**
采用四点或五点校准法，通过线性变换计算触摸坐标与LCD坐标的映射关系。

校准参数存储在 EEPROM 中，包括：
- X方向增益系数
- Y方向增益系数
- X偏移量
- Y偏移量

```c
// 触摸校准主函数
void tp_adjust(void)
{
    uint16_t pos_temp[4][2];  // 坐标缓存
    
    // 1. 在四个角显示校准标记
    tp_draw_touch_point(20, 20, RED);
    // 等待用户触摸...
    
    // 2. 计算校准参数
    tp_dev.xfac = (float)(lcddev.width - 40) / (pos_temp[1][0] - pos_temp[0][0]);
    tp_dev.yfac = (float)(lcddev.height - 40) / (pos_temp[3][1] - pos_temp[0][1]);
    
    // 3. 保存到EEPROM
    at24cxx_write(0, (uint8_t*)&tp_dev, sizeof(_m_tp_dev));
}
```

---

### 三、系统架构设计

#### 3.1 分层架构

```
┌─────────────────────────────────────┐
│      应用层 (main.c)                │  用户程序
│  - 触摸事件处理                      │
│  - UI绘制                           │
├─────────────────────────────────────┤
│      BSP层 (Board Support Package) │  驱动抽象层
│  - touch.c  (统一触摸接口)          │
│  - lcd.c    (统一LCD接口)           │
├─────────────────────────────────────┤
│      硬件驱动层                      │  具体驱动实现
│  - gt9xxx.c / ft5206.c (触摸驱动)  │
│  - lcd_ex.c (LCD芯片驱动)           │
│  - ctiic.c  (IIC总线驱动)           │
├─────────────────────────────────────┤
│      HAL层 (Hardware Abstraction)  │  STM32 HAL库
│  - FSMC / GPIO / IIC               │
├─────────────────────────────────────┤
│      硬件层                          │  STM32F407芯片
│  - CPU / Memory / Peripherals      │
└─────────────────────────────────────┘
```

**设计优势：**
1. **可移植性**：更换LCD或触摸芯片只需修改驱动层，不影响应用层
2. **可扩展性**：轻松添加新的LCD/触摸芯片支持
3. **易维护性**：层次清晰，便于调试和维护

#### 3.2 触摸设备抽象

使用结构体和函数指针实现触摸设备的抽象：

```c
typedef struct
{
    uint8_t (*init)(void);          // 初始化函数指针
    uint8_t (*scan)(uint8_t);       // 扫描函数指针
    void (*adjust)(void);           // 校准函数指针
    
    uint16_t x[10];                 // 触摸点X坐标（支持10点）
    uint16_t y[10];                 // 触摸点Y坐标
    uint16_t sta;                   // 触摸状态
    
    float xfac;                     // X校准系数
    float yfac;                     // Y校准系数
    short xoff;                     // X偏移量
    short yoff;                     // Y偏移量
    
    uint8_t touchtype;              // 触摸类型标记
} _m_tp_dev;

extern _m_tp_dev tp_dev;  // 全局触摸设备对象
```

使用方式：
```c
tp_dev.init();          // 初始化（自动识别芯片）
tp_dev.scan(0);         // 扫描触摸（调用对应芯片的扫描函数）
uint16_t x = tp_dev.x[0];  // 获取坐标
```

#### 3.3 性能优化

**1. FSMC高速访问：**
- 直接内存映射，读写LCD如同操作内存
- 无需软件模拟时序，充分发挥MCU性能
- 典型帧率可达 30-60 FPS（480×800分辨率）

**2. DMA传输（可选扩展）：**
虽然本项目未实现，但FSMC支持DMA：
```c
// 大批量数据传输可使用DMA加速
HAL_DMA_Start(&hdma, (uint32_t)buffer, (uint32_t)&LCD->LCD_RAM, length);
```

**3. 触摸扫描优化：**
- 主循环 10ms 扫描一次（100Hz采样率）
- 电容屏自带中断信号，可改为中断驱动减少CPU占用

---

### 四、关键技术点

#### 4.1 FSMC地址线映射技巧

**为什么使用 FSMC_A6？**
- A0-A15 对应地址偏移的第1-16位
- A6 对应偏移 = (1 << 6+1) = 0x80
- 形成两个连续的16位地址，方便访问：
  ```
  LCD_REG = 0x6C000000 + (0x80 << 0) = 0x6C000000  (A6=0)
  LCD_RAM = 0x6C000000 + (0x80 << 1) = 0x6C000100  (A6=1)
  ```

#### 4.2 多芯片兼容性设计

通过读取 LCD ID 自动适配不同芯片：
```c
uint16_t lcd_read_id(void)
{
    lcd_wr_regno(0xD3);  // 读ID命令
    lcd_rd_data();       // dummy read
    lcd_rd_data();       // 厂商ID
    uint16_t id = lcd_rd_data();  // 芯片ID高字节
    id = (id << 8) | lcd_rd_data();  // 芯片ID低字节
    return id;
}
```

支持的芯片：
- `0x9341`: ILI9341
- `0x5310`: NT35310
- `0x5510`: NT35510
- `0x1963`: SSD1963
- `0x7789`: ST7789

#### 4.3 坐标系转换

支持横竖屏切换，需要进行坐标转换：
```c
// 横屏模式（width > height）
if(lcddev.dir == 1)
{
    temp = tp_dev.x[0];
    tp_dev.x[0] = tp_dev.y[0];
    tp_dev.y[0] = lcddev.height - temp;
}

// 触摸方向翻转
if(tp_dev.touchtype & 0x01)
{
    tp_dev.x[0] = lcddev.width - tp_dev.x[0];
    tp_dev.y[0] = lcddev.height - tp_dev.y[0];
}
```

---

## 常见问题 FAQ

### Q1: LCD显示异常或花屏？
**A:** 
1. 检查 FSMC 时序配置是否匹配LCD芯片数据手册
2. 确认 LCD ID 读取正确，初始化序列对应正确的芯片
3. 检查硬件连接，特别是数据线 D0-D15 的连接
4. 确认供电电压稳定（3.3V）

### Q2: 触摸无响应？
**A:**
1. 检查 IIC 总线通信是否正常（示波器或逻辑分析仪）
2. 确认触摸芯片电源和复位信号正常
3. 检查 INT 中断引脚是否有低电平脉冲
4. 尝试读取触摸芯片 ID 确认通信正常

### Q3: 触摸坐标不准？
**A:**
1. 电容屏：检查坐标转换代码是否匹配LCD方向
2. 电阻屏：执行触摸校准 `tp_adjust()`
3. 确认触摸分辨率与LCD分辨率的映射关系

### Q4: 如何移植到其他STM32芯片？
**A:**
1. 使用 STM32CubeMX 重新配置 FSMC 和 GPIO
2. 修改 `lcd.h` 中的引脚定义
3. 修改 `touch.h` 中的引脚定义
4. 重新生成代码并编译

---

## 参考资料

1. **STM32F407 参考手册**: [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
2. **FSMC控制器应用笔记**: [AN2784](https://www.st.com/resource/en/application_note/cd00201397.pdf)
3. **GT9147 数据手册**: Goodix 官网
4. **FT5206 数据手册**: FocalTech 官网
5. **ILI9341 数据手册**: ILItek 官网

---

## 开源协议

本项目遵循 MIT License，详见 [LICENSE](LICENSE) 文件。

---

## 技术支持

- **原作者**: 正点原子 (ALIENTEK)
- **官方网站**: [www.alientek.com](http://www.alientek.com)
- **技术论坛**: [www.openedv.com](http://www.openedv.com)
- **在线商城**: [openedv.taobao.com](https://openedv.taobao.com)

---

## 版本历史

- **V1.0** (2023-11-xx): 初始版本发布
  - 支持 STM32F407 + TFT-LCD (480×800)
  - 支持 GT9xxx / FT5206 电容触摸
  - 支持电阻式触摸屏
  - 完整的BSP驱动架构

---

## 致谢

感谢正点原子团队提供的优秀开发板和技术支持！
