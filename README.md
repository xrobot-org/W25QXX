# W25QXX

W25QXX FLASH 驱动 / W25QXX Flash Driver

---

## 简介 / Introduction

本模块是基于 LibXR 框架的 W25QXX SPI NOR Flash 驱动，支持 16Mbit ~ 1Gbit 全系列芯片。自动识别容量。

This module implements a flexible W25QXX SPI NOR Flash driver (C++ template), supporting all common capacities. Capacity is auto-detected by JEDEC ID.  

---

## 依赖硬件 / Required Hardware

- `spi_w25qxx`：SPI 控制器（硬件 SPI 句柄）
- `spi_w25qxx_cs`：片选引脚（GPIO 句柄）

---

## 构造参数 / Constructor Arguments

- 无 None

---

## 模板参数 / Template Arguments

- `BUFFER_SIZE`（默认 128）：操作缓冲区大小。默认值足以覆盖绝大多数用例，建议与主芯片 RAM 能力匹配。
  - `W25QXX<256> flash(hw, app_mgr);`

---

## 依赖模块 / Depends

- 无 None

---

## 支持容量 / Supported Capacity

- 2MB (16Mbit), 4MB (32Mbit), 8MB (64Mbit), 16MB (128Mbit), 32MB (256Mbit), 64MB (512Mbit), 128MB (1Gbit)

---

## 特性 / Features

- 支持 Winbond W25Qxx 所有主流 SPI Flash 型号
- JEDEC ID 自动容量识别
- 支持页编程、4K/32K/64K 擦除和整片擦除
- 线程安全的 SPI 操作（带信号量）
- 数据库封装：自动注册为"database" (LibXR::DatabaseRaw)
- 可配置缓冲区大小
- 快速读、状态寄存器访问、掉电/唤醒/复位
- 易于集成，接口与 LibXR::Flash/Database 兼容
