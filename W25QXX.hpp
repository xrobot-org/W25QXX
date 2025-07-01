#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: W25QXX FLASH 驱动 / W25QXX flash driver
constructor_args: []
template_args: 
  - buffer_size: 128
required_hardware:
  - spi_w25qxx
  - spi_w25qxx_cs
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "database.hpp"
#include "flash.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "logger.hpp"
#include "semaphore.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "timebase.hpp"
template <unsigned int BUFFER_SIZE = 128> class W25QXX : public LibXR::Application {
public:
  enum class Command : uint8_t {
    WriteEnable = 0x06,         // 写使能
    WriteDisable = 0x04,        // 写禁止
    ReadStatusReg1 = 0x05,      // 读状态寄存器1
    ReadStatusReg2 = 0x35,      // 读状态寄存器2
    ReadStatusReg3 = 0x15,      // 读状态寄存器3
    WriteStatusReg = 0x01,      // 写状态寄存器
    ReadData = 0x03,            // 读数据
    FastRead = 0x0B,            // 快速读
    PageProgram = 0x02,         // 页编程
    SectorErase = 0x20,         // 擦除4KB扇区
    BlockErase32K = 0x52,       // 擦除32KB块
    BlockErase64K = 0xD8,       // 擦除64KB块
    ChipErase = 0xC7,           // 整片擦除
    ReadJedecId = 0x9F,         // 读JEDEC ID
    ReadUniqId = 0x4B,          // 读唯一ID
    ReadManufacturerDev = 0x90, // 读厂家/设备ID
    PowerDown = 0xB9,           // 掉电
    ReleasePowerDown = 0xAB,    // 唤醒
    Enable4ByteAddr = 0xB7,     // 进入4字节地址模式
    Exit4ByteAddr = 0xE9,       // 退出4字节地址模式
    ResetEnable = 0x66,         // 复位使能
    Reset = 0x99,               // 复位
  };

  enum class Capacity : uint8_t {
    UNKNOWN = 0x00,
    M_2MB = 0x15,   // 16Mbit
    M_4MB = 0x16,   // 32Mbit
    M_8MB = 0x17,   // 64Mbit
    M_16MB = 0x18,  // 128Mbit
    M_32MB = 0x19,  // 256Mbit
    M_64MB = 0x20,  // 512Mbit
    M_128MB = 0x21, // 1Gbit
  };

  class FlashWrapper : public LibXR::Flash {
  public:
    FlashWrapper(W25QXX &w25qxx)
        : LibXR::Flash(4 * 1024, 1, LibXR::RawData(nullptr, w25qxx.capacity_)),
          w25qxx_(&w25qxx) {}

    ErrorCode Erase(size_t offset, size_t size) override {
      return w25qxx_->Erase(offset, size);
    }

    ErrorCode Write(size_t offset, LibXR::ConstRawData data) override {
      return w25qxx_->PageProgramAuto(
          offset, reinterpret_cast<const uint8_t *>(data.addr_), data.size_);
    }

    ErrorCode Read(size_t offset, LibXR::RawData data) override {
      return w25qxx_->Read(offset, reinterpret_cast<uint8_t *>(data.addr_),
                           data.size_);
    }

  private:
    W25QXX *w25qxx_;
  };

  W25QXX(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    UNUSED(app);
    spi_ = hw.template FindOrExit<LibXR::SPI>({"spi_w25qxx"});
    spi_cs_ = hw.template FindOrExit<LibXR::GPIO>({"spi_w25qxx_cs"});

    spi_->SetConfig({.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
                     .clock_phase = LibXR::SPI::ClockPhase::EDGE_1});

    spi_cs_->SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                        .pull = LibXR::GPIO::Pull::NONE});
    spi_cs_->Write(true);

    auto ans = Init();
    while (!ans) {
      XR_LOG_ERROR("W25QXX init failed");
      LibXR::Thread::Sleep(50);
      ans = Init();
    }

    flash_ = new FlashWrapper(*this);
    db_ = new LibXR::DatabaseRaw<1>(*flash_);

    hw.Register(LibXR::Entry<LibXR::Flash>{*flash_, {"flash"}});
    hw.Register(LibXR::Entry<LibXR::DatabaseRaw<1>>{*db_, {"database"}});
  }

  bool Init() {
    Reset();
    LibXR::Thread::Sleep(5);
    ReadCmd(Command::ReadJedecId, {&id_[0], 3});
    switch (static_cast<Capacity>(id_[2])) {
    case Capacity::M_2MB:
      capacity_ = 1024 * 1024 * 2;
      break;
    case Capacity::M_4MB:
      capacity_ = 1024 * 1024 * 4;
      break;
    case Capacity::M_8MB:
      capacity_ = 1024 * 1024 * 8;
      break;
    case Capacity::M_16MB:
      capacity_ = 1024 * 1024 * 16;
      break;
    case Capacity::M_32MB:
      capacity_ = 1024 * 1024 * 32;
      break;
    case Capacity::M_64MB:
      capacity_ = 1024 * 1024 * 64;
      break;
    case Capacity::M_128MB:
      capacity_ = 1024 * 1024 * 128;
      break;
    default:
      return false;
    }

    return true;
  }

  ErrorCode WriteCmd(Command cmd, LibXR::ConstRawData data) {
    spi_cs_->Write(false);
    write_buffer_[0] = static_cast<uint8_t>(cmd);
    memcpy(write_buffer_ + 1, data.addr_, data.size_);
    auto ans = spi_->Write({write_buffer_, data.size_ + 1}, spi_op_);
    spi_cs_->Write(true);
    return ans;
  }

  ErrorCode ReadCmd(Command cmd, LibXR::RawData data) {
    spi_cs_->Write(false);
    write_buffer_[0] = static_cast<uint8_t>(cmd);
    auto ans = spi_->ReadAndWrite({read_buffer_, data.size_ + 1},
                                  {write_buffer_, data.size_ + 1}, spi_op_);
    spi_cs_->Write(true);
    memcpy(data.addr_, read_buffer_ + 1, data.size_);
    return ans;
  }

  ErrorCode FastRead(uint32_t addr, uint8_t *buf, size_t len) {
    ASSERT(len <= BUFFER_SIZE);
    write_buffer_[0] = static_cast<uint8_t>(Command::FastRead);
    write_buffer_[1] = static_cast<uint8_t>(addr >> 16);
    write_buffer_[2] = static_cast<uint8_t>(addr >> 8);
    write_buffer_[3] = static_cast<uint8_t>(addr >> 0);
    write_buffer_[4] = 0x00;

    spi_cs_->Write(false);
    auto ans = spi_->ReadAndWrite({read_buffer_, len + 5},
                                  {write_buffer_, len + 5}, spi_op_);
    spi_cs_->Write(true);
    memcpy(buf, read_buffer_ + 5, len);
    return ans;
  }

  ErrorCode Read(uint32_t addr, uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i += BUFFER_SIZE) {
      auto remain = LibXR::min(BUFFER_SIZE, len - i);
      auto ans = FastRead(addr + i, buf + i, remain);
      if (ans != ErrorCode::OK)
        return ans;
    }
    return ErrorCode::OK;
  }

  ErrorCode PageProgramAuto(uint32_t addr, const uint8_t *buf, size_t len) {
    size_t page_size = 256;
    size_t remain = len;
    size_t offset = 0;

    while (remain > 0) {
      size_t page_offset = addr % page_size;
      size_t write_len = std::min(page_size - page_offset, remain);

      auto ans = PageProgram(addr, buf + offset, write_len);
      if (ans != ErrorCode::OK)
        return ans;

      addr += write_len;
      offset += write_len;
      remain -= write_len;
    }
    return ErrorCode::OK;
  }

  ErrorCode PageProgram(uint32_t addr, const uint8_t *buf, size_t len) {
    ASSERT(len <= BUFFER_SIZE);

    // 先写使能
    WriteEnable();

    // 构建写指令
    write_buffer_[0] = static_cast<uint8_t>(Command::PageProgram);
    write_buffer_[1] = static_cast<uint8_t>(addr >> 16);
    write_buffer_[2] = static_cast<uint8_t>(addr >> 8);
    write_buffer_[3] = static_cast<uint8_t>(addr >> 0);
    memcpy(write_buffer_ + 4, buf, len);

    spi_cs_->Write(false);
    auto ans = spi_->Write({write_buffer_, len + 4}, spi_op_);
    spi_cs_->Write(true);

    WaitBusy(1, 10);

    return ans;
  }

  ErrorCode WriteEnable() {
    write_buffer_[0] = static_cast<uint8_t>(Command::WriteEnable);
    spi_cs_->Write(false);
    auto ans = spi_->Write({write_buffer_, 1}, spi_op_);
    spi_cs_->Write(true);
    return ans;
  }

  bool IsBusy() {
    uint8_t status = 0;
    ReadCmd(Command::ReadStatusReg1, {&status, 1});
    return status & 0x01;
  }

  void WaitBusy(uint32_t cycle = 1, uint32_t timeout = 2000) {
    auto start = LibXR::Timebase::GetMilliseconds();
    while (IsBusy()) {
      LibXR::Thread::Sleep(cycle);
      if (LibXR::Timebase::GetMilliseconds() - start > timeout)
        return;
    }
  }

  enum class EraseType { Sector4K, Block32K, Block64K };

  ErrorCode Erase(uint32_t addr, size_t size) {
    if ((size % (4 * 1024)) != 0)
      return ErrorCode::ARG_ERR;

    while (size > 0) {
      if ((size >= 64 * 1024) && ((addr % (64 * 1024)) == 0)) {
        auto ans = EraseBlock(addr, EraseType::Block64K);
        if (ans != ErrorCode::OK)
          return ans;
        addr += 64 * 1024;
        size -= 64 * 1024;
      } else if ((size >= 32 * 1024) && ((addr % (32 * 1024)) == 0)) {
        auto ans = EraseBlock(addr, EraseType::Block32K);
        if (ans != ErrorCode::OK)
          return ans;
        addr += 32 * 1024;
        size -= 32 * 1024;
      } else if ((size >= 4 * 1024) && ((addr % (4 * 1024)) == 0)) {
        auto ans = EraseBlock(addr, EraseType::Sector4K);
        if (ans != ErrorCode::OK)
          return ans;
        addr += 4 * 1024;
        size -= 4 * 1024;
      } else {
        return ErrorCode::ARG_ERR;
      }
    }
    return ErrorCode::OK;
  }

  ErrorCode EraseBlock(uint32_t addr, EraseType type) {
    WriteEnable();
    uint32_t timeout = 1000;
    switch (type) {
    case EraseType::Sector4K:
      write_buffer_[0] = static_cast<uint8_t>(Command::SectorErase);
      timeout = 500;
      break;
    case EraseType::Block32K:
      write_buffer_[0] = static_cast<uint8_t>(Command::BlockErase32K);
      timeout = 2000;
      break;
    case EraseType::Block64K:
      write_buffer_[0] = static_cast<uint8_t>(Command::BlockErase64K);
      timeout = 2500;
      break;
    }
    write_buffer_[1] = (addr >> 16) & 0xFF;
    write_buffer_[2] = (addr >> 8) & 0xFF;
    write_buffer_[3] = addr & 0xFF;

    spi_cs_->Write(false);
    auto ans = spi_->Write({write_buffer_, 4}, spi_op_);
    spi_cs_->Write(true);
    WaitBusy(100, timeout);
    return ans;
  }

  void Reset() { WriteCmd(Command::Reset, {}); }

  void ChipErase() {
    WriteEnable();
    WriteCmd(Command::ChipErase, {});
    WaitBusy(1000, 25000);
  }

  void OnMonitor() override {}

private:
  uint8_t id_[3] = {0};
  size_t capacity_ = 0;
  LibXR::DatabaseRaw<1> *db_;
  LibXR::Flash *flash_;
  LibXR::SPI *spi_;
  LibXR::GPIO *spi_cs_;

  uint8_t read_buffer_[BUFFER_SIZE + 5], write_buffer_[BUFFER_SIZE + 5];

  LibXR::Semaphore spi_sem_;
  LibXR::SPI::OperationRW spi_op_ = LibXR::SPI::OperationRW(spi_sem_, 32);
};
