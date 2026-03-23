# STM32 + FreeRTOS + TB6600 开发笔记

## 1. FreeRTOS RAM 溢出问题

### 现象
```
ld.exe: section `._user_heap_stack' will not fit in region `RAM'
ld.exe: region `RAM' overflowed by 104 bytes
```

### 原因
`configTOTAL_HEAP_SIZE` 设置过大。FreeRTOS 的 heap（`ucHeap[]` 数组）位于 `.bss` 段，
直接占用 SRAM。链接器会检查：

```
.data + .bss（含 ucHeap）+ _Min_Heap_Size(0x200) + _Min_Stack_Size(0x400) ≤ RAM 总量
```

STM32F103C6 只有 **10 KB SRAM**，将 heap 从 3072 改为 5120 后超限。

### 计算方法
```
最大可用 heap = RAM总量 - 其他.bss/.data - 链接器预留
             = 10240 - 3688 - 1536 = 5016 字节
```

### 解决方案
```c
// FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE       ((size_t)4800)  // 留 216 字节链接余量
#define configTIMER_TASK_STACK_DEPTH 128             // 从 256 降低，节省 512 字节运行时 heap
```

### 各任务 heap 占用估算（6个任务）
| 项目 | 大小 |
|---|---|
| 4 个用户任务栈（各 128×4 = 512B）| 2048 B |
| Timer 任务栈（128×4）| 512 B |
| Idle 任务栈（128×4）| 512 B |
| 6 个 TCB（各 ~88B）| 528 B |
| 3 个信号量（各 ~80B）| 240 B |
| 队列等杂项 | ~200 B |
| **合计** | **~4040 B** |

---

## 2. PWM 启动代码放在 osKernelStart() 之后导致电机不转

### 现象
电机完全不动，PWM 未输出。

### 原因
`osKernelStart()` 不会返回，其后的代码**永远不会执行**。
CubeMX 自动生成时将 PWM 启动代码放在了 `while(1)` 前但位于调度器启动之后。

```c
osKernelStart();   // ← 调度器接管，不返回

// ↓ 死代码，永远不执行
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
```

### 解决方案
将 PWM 初始化移到 `osKernelStart()` **之前**的 `USER CODE BEGIN 2` 区域：

```c
/* USER CODE BEGIN 2 */
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
/* USER CODE END 2 */

osKernelInitialize();
MX_FREERTOS_Init();
osKernelStart();  // 之后的代码不可达
```

---

## 3. CubeMX 重新生成代码修改了 TIM 参数导致电机不转

### 现象
添加 FreeRTOS 后重新生成代码，电机不转。

### 原因
CubeMX 重新生成时修改了 `tim.c` 中的两个参数：

| 参数 | 原始值（可用）| 重新生成后 |
|---|---|---|
| `Period` | **999** | 99 |
| `OCMode` | **TIM_OCMODE_PWM2** | TIM_OCMODE_PWM1 |

Period 从 999 改为 99 后，Compare=500 **超出范围**（0~99），
在 PWM1 模式下计数器永远小于 Compare，输出恒为高电平，不产生脉冲边沿。

### 解决方案
手动恢复 `tim.c` 中的原始参数：

```c
htim2.Init.Period = 999;           // 恢复，对应 1 kHz PWM
sConfigOC.OCMode = TIM_OCMODE_PWM2; // 恢复
```

> **注意**：每次用 CubeMX 重新生成代码后，需检查 `tim.c` 参数是否被改动。
> 建议将关键参数记录在此文档中作为对照。

---

## 4. TB6600 步进电机驱动 PWM 频率选择

### PWM1 vs PWM2
两者占空比相同，仅输出极性相反（高低电平翻转）。
对 TB6600 而言本质无差别，但需与实际接线匹配。

### 频率对电机的影响

| 配置 | PWM 频率 | 1/16 细分下转速 |
|---|---|---|
| Period=999 | 1 kHz | ~62 转/秒 |
| Period=99  | 10 kHz | ~625 转/秒（上电直接失步）|

### 结论
**上电时不能直接输出高频 PWM**，步进电机需要从低速起步，否则失步/堵转。
TB6600 推荐起步频率：≤ 1 kHz，之后通过加速曲线逐步提高。

---

## TIM2 参数速查

```
时钟源：APB1 = 72 MHz
Prescaler = 71  →  计数时钟 = 72M ÷ 72 = 1 MHz（每步 1 µs）
Period    = 999 →  PWM 周期 = 1000 µs = 1 ms（1 kHz）
Compare   = 500 →  占空比 50%
```

调速只需修改 Compare 值（范围 0~999）：
```c
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500); // 50% 占空比
```