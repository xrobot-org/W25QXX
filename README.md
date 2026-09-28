# W25QXX

W25QXX FLASH 驱动：基于 LibXR 的 Winbond W25Qxx SPI NOR Flash 驱动（C++ 模板），
按 JEDEC ID 自动识别容量，并提供 `LibXR::Flash` 视图和一个 `LibXR::DatabaseRaw`。

W25QXX flash driver: a LibXR driver (C++ template) for Winbond W25Qxx SPI NOR
flash. It detects the capacity from the JEDEC ID and provides `LibXR::Flash` views
and a `LibXR::DatabaseRaw`.

## 行为 / Behaviour

- 构造时把 SPI 设为 CPOL 低 / 第一边沿采样，把 `cs` 设为推挽输出并拉高，然后复位芯片、
  读取 JEDEC ID。容量无法识别时打印 `W25QXX init failed` 并每 50 ms 重试，直到成功
  （构造会一直阻塞）。
- 支持容量 / Supported capacity：2 MB (16 Mbit)、4 MB (32 Mbit)、8 MB (64 Mbit)、
  16 MB (128 Mbit)、32 MB (256 Mbit)、64 MB (512 Mbit)、128 MB (1 Gbit)。
- 命令只使用 3 字节地址，不进入 4 字节地址模式，因此大于 16 MB 的芯片只能直接访问前
  16 MB。
- 读用 Fast Read，按 `BUFFER_SIZE` 分块；写用页编程，按 256 字节页边界拆分；擦除按
  地址对齐自动选择 64 KB / 32 KB / 4 KB 块，大小必须是 4 KB 的整数倍；另有整片擦除
  `ChipErase()`。
- 单次页编程要求长度不超过 `BUFFER_SIZE`（`ASSERT`）。默认 `BUFFER_SIZE = 128` 时，
  一次写入在同一页内超过 128 字节会触发断言；需要整页写入时使用 `W25QXX<256>`。
- 模块不做互斥：SPI 读写共用内部缓冲区，多个线程并发访问同一实例需要调用方自己加锁。

- The constructor sets the SPI bus to CPOL low / first-edge sampling, configures
  `cs` as a push-pull output driven high, resets the chip and reads the JEDEC ID.
  If the capacity is not recognized it logs `W25QXX init failed` and retries every
  50 ms until it succeeds (construction blocks until then).
- Only 3-byte address commands are used and 4-byte address mode is not entered, so
  on chips larger than 16 MB only the first 16 MB are directly addressable.
- Reads use Fast Read in chunks of `BUFFER_SIZE`; writes use page program, split at
  256-byte page boundaries; erase picks 64 KB / 32 KB / 4 KB blocks from the address
  alignment and requires a size that is a multiple of 4 KB; `ChipErase()` erases the
  whole chip.
- A single page program must not exceed `BUFFER_SIZE` bytes (`ASSERT`). With the
  default `BUFFER_SIZE = 128`, a write of more than 128 bytes within one page fails
  the assertion; use `W25QXX<256>` for full-page writes.
- There is no locking: SPI transfers share internal buffers, so callers must
  serialize concurrent access to one instance.

## 接口 / API

- `LibXR::Flash& GetFlash()`：整片 Flash 视图（最小擦除 4 KB，写粒度 1 字节）。/
  Whole-chip view (4 KB minimum erase, 1-byte write granularity).
- `LibXR::Flash& GetDatabaseFlash()`：芯片末尾 128 KB（芯片更小时为整片）的窗口。/
  Window over the last 128 KB of the chip (the whole chip if smaller).
- `LibXR::DatabaseRaw<1>& GetDatabaseRaw()` / `LibXR::Database& GetDatabase()`：
  建在该窗口上的数据库。/ The database built on that window.
- 底层操作 / Low-level operations：`Read()`、`FastRead()`、`PageProgram()`、
  `PageProgramAuto()`、`Erase()`、`EraseBlock()`、`ChipErase()`、`IsBusy()`、
  `WaitBusy()`、`Reset()`。

模块不会把数据库注册到任何全局名字；需要它的代码通过 `W25QXX<...>&` 获得实例后调用
上述接口。
The Module does not register the database under any global name; code that needs
it takes the `W25QXX<...>&` instance and calls the accessors above.

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

```cpp
template <unsigned int BUFFER_SIZE = 128>
class W25QXX;

W25QXX(LibXR::SPI& spi,
       LibXR::GPIO& cs);
```

模板参数 / Template parameter:

- `BUFFER_SIZE`：SPI 传输缓冲区大小（字节），即单次读取块和单次页编程的最大长度，
  默认 128。/ SPI transfer buffer size in bytes, i.e. the read chunk and the maximum
  single page-program length, default 128.

依赖 / Dependencies:

- `spi`：连接 Flash 的 `LibXR::SPI`。/ The `LibXR::SPI` bus of the flash.
- `cs`：片选 GPIO（低有效）。/ Chip-select GPIO (active low).

无值配置。/ No value configuration.

## 使用 / Use

```sh
xrobot module add xrobot-org/W25QXX
xrobot setup
xrobot instance add xrobot-org/W25QXX
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，模板参数按源码
默认值写出；把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source default template argument; fill the dependencies with
the names of the objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/W25QXX
    id: w25qxx_0
    template_args:
      - '128'
    args:
      - spi: spi2
      - cs: flash_cs
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(spi2, LibXR::SPI);
XR_REGISTER(flash_cs, LibXR::GPIO);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/W25QXX`
（在 BSP 中）打印 manifest 和当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/W25QXX` in a BSP, prints the manifest and
the current constructor.
