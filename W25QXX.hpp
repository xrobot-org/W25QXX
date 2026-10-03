#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Winbond W25Qxx SPI NOR Flash 驱动模块 / Driver module for the Winbond W25Qxx SPI NOR flash
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstring>
#include <memory>

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

/**
 * @brief W25Qxx SPI NOR Flash 驱动，按 JEDEC ID 识别容量，提供 Flash 视图与数据库。
 *        Driver for the Winbond W25Qxx SPI NOR flash; detects the capacity from the
 *        JEDEC ID and provides Flash views and a database.
 *
 * @tparam BUFFER_SIZE SPI 传输缓冲区大小，单位字节，也是单次读取块的大小。
 *                     SPI transfer buffer size in bytes, also the read chunk size.
 */
template <unsigned int BUFFER_SIZE = 128>
class W25QXX
{
 public:
  /**
   * @brief W25Qxx 命令码。
   *        W25Qxx command codes.
   */
  enum class Command : uint8_t
  {
    WriteEnable = 0x06,          ///< 写使能 Write enable
    WriteDisable = 0x04,         ///< 写禁止 Write disable
    ReadStatusReg1 = 0x05,       ///< 读状态寄存器 1 Read status register 1
    ReadStatusReg2 = 0x35,       ///< 读状态寄存器 2 Read status register 2
    ReadStatusReg3 = 0x15,       ///< 读状态寄存器 3 Read status register 3
    WriteStatusReg = 0x01,       ///< 写状态寄存器 Write status register
    ReadData = 0x03,             ///< 读数据 Read data
    FastRead = 0x0B,             ///< 快速读 Fast read
    PageProgram = 0x02,          ///< 页编程 Page program
    SectorErase = 0x20,          ///< 擦除 4 KB 扇区 Erase a 4 KB sector
    BlockErase32K = 0x52,        ///< 擦除 32 KB 块 Erase a 32 KB block
    BlockErase64K = 0xD8,        ///< 擦除 64 KB 块 Erase a 64 KB block
    ChipErase = 0xC7,            ///< 整片擦除 Chip erase
    ReadJedecId = 0x9F,          ///< 读 JEDEC ID Read JEDEC ID
    ReadUniqId = 0x4B,           ///< 读唯一 ID Read unique ID
    ReadManufacturerDev = 0x90,  ///< 读厂家与设备 ID Read manufacturer and device ID
    PowerDown = 0xB9,            ///< 掉电 Power down
    ReleasePowerDown = 0xAB,     ///< 唤醒 Release power down
    Enable4ByteAddr = 0xB7,      ///< 进入 4 字节地址模式 Enter 4-byte address mode
    Exit4ByteAddr = 0xE9,        ///< 退出 4 字节地址模式 Exit 4-byte address mode
    ResetEnable = 0x66,          ///< 复位使能 Reset enable
    Reset = 0x99,                ///< 复位 Reset
  };

  /**
   * @brief JEDEC ID 中的容量码。
   *        Capacity code in the JEDEC ID.
   */
  enum class Capacity : uint8_t
  {
    UNKNOWN = 0x00,  ///< 无法识别 Not recognized
    M_2MB = 0x15,    ///< 2 MB，16 Mbit 2 MB, 16 Mbit
    M_4MB = 0x16,    ///< 4 MB，32 Mbit 4 MB, 32 Mbit
    M_8MB = 0x17,    ///< 8 MB，64 Mbit 8 MB, 64 Mbit
    M_16MB = 0x18,   ///< 16 MB，128 Mbit 16 MB, 128 Mbit
    M_32MB = 0x19,   ///< 32 MB，256 Mbit 32 MB, 256 Mbit
    M_64MB = 0x20,   ///< 64 MB，512 Mbit 64 MB, 512 Mbit
    M_128MB = 0x21,  ///< 128 MB，1 Gbit 128 MB, 1 Gbit
  };

  /**
   * @brief 覆盖整片芯片的 LibXR::Flash 视图。
   *        LibXR::Flash view covering the whole chip.
   */
  class FlashWrapper : public LibXR::Flash
  {
   public:
    /**
     * @brief 构造整片视图，最小擦除 4 KB，写粒度 1 字节。
     *        Construct the whole-chip view with 4 KB minimum erase and 1-byte write
     *        granularity.
     *
     * @param w25qxx 被封装的 W25QXX 实例。
     *               The wrapped W25QXX instance.
     */
    FlashWrapper(W25QXX& w25qxx)
        : LibXR::Flash(4 * 1024, 1, LibXR::RawData(nullptr, w25qxx.capacity_)),
          w25qxx_(&w25qxx)
    {
    }

    LibXR::ErrorCode Erase(size_t offset, size_t size) override
    {
      return w25qxx_->Erase(offset, size);
    }

    LibXR::ErrorCode Write(size_t offset, LibXR::ConstRawData data) override
    {
      return w25qxx_->PageProgramAuto(
          offset, reinterpret_cast<const uint8_t*>(data.addr_), data.size_);
    }

    LibXR::ErrorCode Read(size_t offset, LibXR::RawData data) override
    {
      return w25qxx_->Read(offset, reinterpret_cast<uint8_t*>(data.addr_), data.size_);
    }

   private:
    W25QXX* w25qxx_;
  };

  /**
   * @brief 覆盖芯片一段区间的 LibXR::Flash 视图，偏移相对区间起点。
   *        LibXR::Flash view over a range of the chip; offsets are relative to the start
   *        of the range.
   */
  class FlashWindow : public LibXR::Flash
  {
   public:
    /**
     * @brief 构造区间视图，最小擦除 4 KB，写粒度 1 字节。
     *        Construct the range view with 4 KB minimum erase and 1-byte write
     *        granularity.
     *
     * @param w25qxx 被封装的 W25QXX 实例。
     *               The wrapped W25QXX instance.
     * @param base 区间在芯片内的起始地址。
     *             Start address of the range in the chip.
     * @param size 区间大小，单位字节。
     *             Range size in bytes.
     */
    FlashWindow(W25QXX& w25qxx, size_t base, size_t size)
        : LibXR::Flash(4 * 1024, 1, LibXR::RawData(nullptr, size)),
          w25qxx_(&w25qxx),
          base_(base)
    {
    }

    LibXR::ErrorCode Erase(size_t offset, size_t size) override
    {
      return w25qxx_->Erase(base_ + offset, size);
    }

    LibXR::ErrorCode Write(size_t offset, LibXR::ConstRawData data) override
    {
      return w25qxx_->PageProgramAuto(
          base_ + offset, reinterpret_cast<const uint8_t*>(data.addr_), data.size_);
    }

    LibXR::ErrorCode Read(size_t offset, LibXR::RawData data) override
    {
      return w25qxx_->Read(base_ + offset, reinterpret_cast<uint8_t*>(data.addr_),
                           data.size_);
    }

   private:
    W25QXX* w25qxx_;
    size_t base_;
  };

  /**
   * @brief 构造 W25QXX：配置 SPI 与片选，复位并识别芯片，容量无法识别时每 50 ms 重试。
   *        Construct W25QXX: configure the SPI and the chip select, reset and identify
   *        the chip, retrying every 50 ms while the capacity is not recognized.
   *
   * @param spi 连接 Flash 的 SPI。
   *            SPI bus of the flash.
   * @param cs 片选 GPIO，低电平有效。
   *           Chip-select GPIO, active low.
   */
  W25QXX(
      LibXR::SPI& spi,
      LibXR::GPIO& cs)
  {
    spi_ = std::addressof(spi);
    spi_cs_ = std::addressof(cs);

    spi_->SetConfig({.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
                     .clock_phase = LibXR::SPI::ClockPhase::EDGE_1});

    spi_cs_->SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                        .pull = LibXR::GPIO::Pull::NONE});
    spi_cs_->Write(true);

    auto ans = Init();
    while (!ans)
    {
      XR_LOG_ERROR("W25QXX init failed");
      LibXR::Thread::Sleep(50);
      ans = Init();
    }

    flash_ = new FlashWrapper(*this);
    const size_t database_size =
        capacity_ >= kDatabaseAreaSize ? kDatabaseAreaSize : capacity_;
    database_flash_ = new FlashWindow(*this, capacity_ - database_size, database_size);
    db_ = new LibXR::DatabaseRaw<1>(*database_flash_);
  }

  /**
   * @brief 复位芯片并读取 JEDEC ID，据此设置容量。
   *        Reset the chip and read the JEDEC ID to set the capacity.
   *
   * @return 容量被识别为 true，否则为 false。
   *         true when the capacity is recognized, false otherwise.
   */
  bool Init()
  {
    Reset();
    LibXR::Thread::Sleep(5);
    ReadCmd(Command::ReadJedecId, {&id_[0], 3});
    switch (static_cast<Capacity>(id_[2]))
    {
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

  /**
   * @brief 在一次片选内发送命令码及其后的数据。
   *        Send a command code followed by data within one chip-select period.
   *
   * @param cmd 命令码。
   *            Command code.
   * @param data 命令之后的数据。
   *             Data following the command.
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode WriteCmd(Command cmd, LibXR::ConstRawData data)
  {
    spi_cs_->Write(false);
    write_buffer_[0] = static_cast<uint8_t>(cmd);
    memcpy(write_buffer_ + 1, data.addr_, data.size_);
    auto ans = spi_->Write({write_buffer_, data.size_ + 1}, spi_op_);
    spi_cs_->Write(true);
    return ans;
  }

  /**
   * @brief 发送命令码并读取其后的数据。
   *        Send a command code and read the data that follows.
   *
   * @param cmd 命令码。
   *            Command code.
   * @param data 接收缓冲区，其大小决定读取的字节数。
   *             Receive buffer; its size sets the number of bytes read.
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode ReadCmd(Command cmd, LibXR::RawData data)
  {
    spi_cs_->Write(false);
    write_buffer_[0] = static_cast<uint8_t>(cmd);
    auto ans = spi_->ReadAndWrite({read_buffer_, data.size_ + 1},
                                  {write_buffer_, data.size_ + 1}, spi_op_);
    spi_cs_->Write(true);
    memcpy(data.addr_, read_buffer_ + 1, data.size_);
    return ans;
  }

  /**
   * @brief 用 Fast Read 命令读取一段数据，len 不超过 BUFFER_SIZE。
   *        Read a block with the Fast Read command; len does not exceed BUFFER_SIZE.
   *
   * @param addr 读取起始地址。
   *             Start address.
   * @param buf 接收缓冲区。
   *            Receive buffer.
   * @param len 读取长度，单位字节。
   *            Length in bytes.
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode FastRead(uint32_t addr, uint8_t* buf, size_t len)
  {
    ASSERT(len <= BUFFER_SIZE);
    write_buffer_[0] = static_cast<uint8_t>(Command::FastRead);
    write_buffer_[1] = static_cast<uint8_t>(addr >> 16);
    write_buffer_[2] = static_cast<uint8_t>(addr >> 8);
    write_buffer_[3] = static_cast<uint8_t>(addr >> 0);
    write_buffer_[4] = 0x00;

    spi_cs_->Write(false);
    auto ans =
        spi_->ReadAndWrite({read_buffer_, len + 5}, {write_buffer_, len + 5}, spi_op_);
    spi_cs_->Write(true);
    memcpy(buf, read_buffer_ + 5, len);
    return ans;
  }

  /**
   * @brief 读取任意长度的数据，按 BUFFER_SIZE 分块调用 FastRead。
   *        Read data of any length, calling FastRead in chunks of BUFFER_SIZE.
   *
   * @param addr 读取起始地址。
   *             Start address.
   * @param buf 接收缓冲区。
   *            Receive buffer.
   * @param len 读取长度，单位字节。
   *            Length in bytes.
   * @return 全部读取成功为 OK，否则为第一个失败的结果。
   *         OK when all chunks succeed, otherwise the first failing result.
   */
  LibXR::ErrorCode Read(uint32_t addr, uint8_t* buf, size_t len)
  {
    for (size_t i = 0; i < len; i += BUFFER_SIZE)
    {
      auto remain = LibXR::min(BUFFER_SIZE, len - i);
      auto ans = FastRead(addr + i, buf + i, remain);
      if (ans != LibXR::ErrorCode::OK) return ans;
    }
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 写入任意长度的数据，按 256 字节页边界和 BUFFER_SIZE 拆分后逐段调用 PageProgram。
   *        Write data of any length, split at 256-byte page boundaries and at
   *        BUFFER_SIZE, and passed to PageProgram segment by segment.
   *
   * @param addr 写入起始地址。
   *             Start address.
   * @param buf 待写入的数据。
   *            Data to write.
   * @param len 数据长度，单位字节。
   *            Length in bytes.
   * @return 全部写入成功为 OK，否则为第一个失败的结果。
   *         OK when all segments succeed, otherwise the first failing result.
   */
  LibXR::ErrorCode PageProgramAuto(uint32_t addr, const uint8_t* buf, size_t len)
  {
    size_t page_size = 256;
    size_t remain = len;
    size_t offset = 0;

    while (remain > 0)
    {
      size_t page_offset = addr % page_size;
      size_t write_len = std::min({page_size - page_offset, remain,
                                   static_cast<size_t>(BUFFER_SIZE)});

      auto ans = PageProgram(addr, buf + offset, write_len);
      if (ans != LibXR::ErrorCode::OK) return ans;

      addr += write_len;
      offset += write_len;
      remain -= write_len;
    }
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 页编程：写使能后写入一段数据并等待芯片空闲，len 不超过 BUFFER_SIZE。
   *        Page program: write enable, write one block and wait until the chip is idle;
   *        len does not exceed BUFFER_SIZE.
   *
   * @param addr 写入起始地址。
   *             Start address.
   * @param buf 待写入的数据。
   *            Data to write.
   * @param len 数据长度，单位字节。
   *            Length in bytes.
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode PageProgram(uint32_t addr, const uint8_t* buf, size_t len)
  {
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

  /**
   * @brief 发送写使能命令。
   *        Send the write enable command.
   *
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode WriteEnable()
  {
    write_buffer_[0] = static_cast<uint8_t>(Command::WriteEnable);
    spi_cs_->Write(false);
    auto ans = spi_->Write({write_buffer_, 1}, spi_op_);
    spi_cs_->Write(true);
    return ans;
  }

  /**
   * @brief 读取状态寄存器 1 的忙标志。
   *        Read the busy flag of status register 1.
   *
   * @return 芯片忙为 true。
   *         true when the chip is busy.
   */
  bool IsBusy()
  {
    uint8_t status = 0;
    ReadCmd(Command::ReadStatusReg1, {&status, 1});
    return status & 0x01;
  }

  /**
   * @brief 每隔 cycle 毫秒查询一次忙标志，直到芯片空闲或超过 timeout 毫秒。
   *        Poll the busy flag every cycle milliseconds until the chip is idle or
   *        timeout milliseconds have elapsed.
   *
   * @param cycle 查询间隔，单位 ms。
   *              Polling interval in ms.
   * @param timeout 超时时间，单位 ms。
   *                Timeout in ms.
   */
  void WaitBusy(uint32_t cycle = 1, uint32_t timeout = 2000)
  {
    auto start = LibXR::Timebase::GetMilliseconds();
    while (IsBusy())
    {
      LibXR::Thread::Sleep(cycle);
      if (LibXR::Timebase::GetMilliseconds() - start > timeout) return;
    }
  }

  /**
   * @brief 擦除块的大小。
   *        Erase block size.
   */
  enum class EraseType
  {
    Sector4K,  ///< 4 KB 扇区 4 KB sector
    Block32K,  ///< 32 KB 块 32 KB block
    Block64K   ///< 64 KB 块 64 KB block
  };

  /**
   * @brief 擦除一段区间，按地址对齐依次选用 64 KB、32 KB、4 KB 块。
   *        Erase a range, using 64 KB, 32 KB and 4 KB blocks according to the address
   *        alignment.
   *
   * @param addr 起始地址，须按 4 KB 对齐。
   *             Start address; must be 4 KB aligned.
   * @param size 区间大小，须为 4 KB 的整数倍。
   *             Range size; must be a multiple of 4 KB.
   * @return 成功为 OK，addr 或 size 不满足对齐要求时为 ARG_ERR。
   *         OK on success, ARG_ERR when addr or size does not meet the alignment.
   */
  LibXR::ErrorCode Erase(uint32_t addr, size_t size)
  {
    if ((size % (4 * 1024)) != 0) return LibXR::ErrorCode::ARG_ERR;

    while (size > 0)
    {
      if ((size >= 64 * 1024) && ((addr % (64 * 1024)) == 0))
      {
        auto ans = EraseBlock(addr, EraseType::Block64K);
        if (ans != LibXR::ErrorCode::OK) return ans;
        addr += 64 * 1024;
        size -= 64 * 1024;
      }
      else if ((size >= 32 * 1024) && ((addr % (32 * 1024)) == 0))
      {
        auto ans = EraseBlock(addr, EraseType::Block32K);
        if (ans != LibXR::ErrorCode::OK) return ans;
        addr += 32 * 1024;
        size -= 32 * 1024;
      }
      else if ((size >= 4 * 1024) && ((addr % (4 * 1024)) == 0))
      {
        auto ans = EraseBlock(addr, EraseType::Sector4K);
        if (ans != LibXR::ErrorCode::OK) return ans;
        addr += 4 * 1024;
        size -= 4 * 1024;
      }
      else
      {
        return LibXR::ErrorCode::ARG_ERR;
      }
    }
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 擦除一个块并等待芯片空闲。
   *        Erase one block and wait until the chip is idle.
   *
   * @param addr 块的起始地址。
   *             Start address of the block.
   * @param type 块的大小。
   *             Block size.
   * @return SPI 传输的结果。
   *         Result of the SPI transfer.
   */
  LibXR::ErrorCode EraseBlock(uint32_t addr, EraseType type)
  {
    WriteEnable();
    uint32_t timeout = 1000;
    switch (type)
    {
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

  /**
   * @brief 发送复位命令。
   *        Send the reset command.
   */
  void Reset() { WriteCmd(Command::Reset, {}); }

  /**
   * @brief 整片擦除并等待芯片空闲，最长等待 25 s。
   *        Erase the whole chip and wait until the chip is idle, for up to 25 s.
   */
  void ChipErase()
  {
    WriteEnable();
    WriteCmd(Command::ChipErase, {});
    WaitBusy(1000, 25000);
  }

  /**
   * @brief 获取整片 Flash 视图。
   *        Get the whole-chip Flash view.
   *
   * @return Flash 视图的引用，由本模块持有。
   *         Reference to the Flash view, owned by this module.
   */
  LibXR::Flash& GetFlash() { return *flash_; }

  /**
   * @brief 获取芯片末尾 128 KB 的 Flash 窗口，芯片更小时为整片。
   *        Get the Flash window over the last 128 KB of the chip, the whole chip if it
   *        is smaller.
   *
   * @return Flash 窗口的引用，由本模块持有。
   *         Reference to the Flash window, owned by this module.
   */
  LibXR::Flash& GetDatabaseFlash() { return *database_flash_; }

  /**
   * @brief 获取建在数据库窗口上的 DatabaseRaw。
   *        Get the DatabaseRaw built on the database window.
   *
   * @return DatabaseRaw 的引用，由本模块持有。
   *         Reference to the DatabaseRaw, owned by this module.
   */
  LibXR::DatabaseRaw<1>& GetDatabaseRaw() { return *db_; }

  /**
   * @brief 获取建在数据库窗口上的 Database 接口。
   *        Get the Database interface built on the database window.
   *
   * @return Database 的引用，由本模块持有。
   *         Reference to the Database, owned by this module.
   */
  LibXR::Database& GetDatabase() { return *db_; }

 private:
  static constexpr size_t kDatabaseAreaSize = 128 * 1024;

  uint8_t id_[3] = {0};
  size_t capacity_ = 0;
  LibXR::DatabaseRaw<1>* db_ = nullptr;
  LibXR::Flash* flash_ = nullptr;
  LibXR::Flash* database_flash_ = nullptr;
  LibXR::SPI* spi_;
  LibXR::GPIO* spi_cs_;

  uint8_t read_buffer_[BUFFER_SIZE + 5], write_buffer_[BUFFER_SIZE + 5];

  LibXR::Semaphore spi_sem_;
  LibXR::SPI::OperationRW spi_op_ = LibXR::SPI::OperationRW(spi_sem_, 32);
};
