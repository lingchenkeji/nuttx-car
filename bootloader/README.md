# STM32F103 USB DFU Bootloader

裸机实现的 USB DFU Bootloader，用于 STM32F103C8T6 (Blue Pill)。

## Flash 布局

```
0x08000000 ┌─────────────────────────┐
           │     Bootloader (12KB)   │
           │                         │
0x08003000 ├─────────────────────────┤
           │     Application (116KB) │
           │       (NuttX APP)       │
           │                         │
0x0801FFFF └─────────────────────────┘
```

## 功能

- **DFU 按键检测 (PA15)**: 上电时按住 PA15（拉低）进入 DFU 模式
- **USB DFU 协议**: 符合 DFU 1.1 规范
- **应用跳转**: 如果 PA0 未按下，自动跳转到应用程序

## 编译

```bash
cd /home/nbclass/nuttxspace/bootloader
make
```

## 烧录 Bootloader

```bash
# 使用 ST-Link
st-flash write bootloader.bin 0x08000000

# 或使用 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "program bootloader.bin 0x08000000 verify reset exit"
```

## 使用 DFU 模式更新应用

1. 按住 PA0 按键
2. 连接 USB
3. 松开 PA0
4. 使用 dfu-util 烧录应用：

```bash
dfu-util -a 0 -D nuttx.bin
```

## 目录结构

```
bootloader/
├── Makefile                    # 编译脚本
├── stm32f103_bootloader.ld     # 链接脚本
├── inc/
│   └── stm32f103.h            # STM32 寄存器定义
└── src/
    ├── startup.c              # 启动代码和向量表
    ├── main.c                 # 主程序
    ├── system.c               # 时钟初始化
    ├── gpio.c                 # GPIO 驱动
    ├── flash.c                # Flash 驱动
    └── usb_dfu.c              # USB DFU 协议实现
```

## 配套的 NuttX APP

NuttX APP 需要使用偏移的链接脚本：

```bash
# 使用 app 配置
cd /home/nbclass/nuttxspace/nuttx
./tools/configure.sh stm32f103-minimum:app

# 编译
make

# 使用 DFU 烧录
dfu-util -a 0 -D nuttx.bin
```

## 注意事项

1. **首次烧录**: 需要通过 ST-Link/SWD 烧录 Bootloader
2. **后续更新**: 可通过 USB DFU 更新应用程序
3. **Bootloader 保护**: Flash 驱动会拒绝写入 Bootloader 区域（0x08000000 - 0x08002FFF）
