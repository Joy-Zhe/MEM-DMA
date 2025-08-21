# MEM-DMA

## Arm侧驱动

1. 仅进行BAR0映射表的配置
``` bash
make arm
```

2. 进行Arm2Host的性能测试
``` bash
make armtest
```

## Host侧驱动

1. 仅进行DMA内存的分配与设备的使能
``` bash
make host
```

2. 打开debug打印线程
``` bash
make hostdebug
```