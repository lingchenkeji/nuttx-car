# 工训电路

本工程以 NuttX 为基础平台搭建而成，是当前默认的公开版本。我们自己的程序在 `apps/myprojects/`，可执行代码与当前版本一致。

主体逻辑、RFID 和语音播报在这里：

- 主体逻辑：`apps/myprojects/projects/project1/project1_main.c`
- RFID：`apps/myprojects/modules/rfid/`
- 语音播报：`apps/myprojects/modules/vtx/`

`project1_main.c` 开头有导读。

板上的配置目录：`nuttx/boards/arm/stm32/stm32f103-minimum/configs/project1/`

电脑端命令格式：`apps/myprojects/projects/project1/CDC_JSON_PROTOCOL.md`
