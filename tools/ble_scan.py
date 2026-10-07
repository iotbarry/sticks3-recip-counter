"""BLE 扫描小工具：确认 StickS3-Recip 是否在广播，并可读取实时状态。

用法:
    python tools/ble_scan.py            # 扫描 8 秒，列出设备
    python tools/ble_scan.py --gatt     # 连上设备并枚举全部服务 / 特征
    python tools/ble_scan.py --watch    # 连上设备并打印实时状态 JSON
依赖: pip install bleak
"""
import asyncio
import sys

TARGET = "StickS3-Recip"
SERVICE = "7a5f1000-3c17-4a5e-b6a1-2d9c8e4f6a01"
CH_STATUS = "7a5f1001-3c17-4a5e-b6a1-2d9c8e4f6a01"
CH_CMD = "7a5f1002-3c17-4a5e-b6a1-2d9c8e4f6a01"


async def scan(seconds: float = 8.0):
    from bleak import BleakScanner
    print(f"扫描 {seconds:.0f} 秒 …")
    res = await BleakScanner.discover(timeout=seconds, return_adv=True)
    # bleak 版本差异: 可能是 dict{addr:(dev,adv)} 或 list[(dev,adv)]
    items = list(res.values()) if isinstance(res, dict) else list(res)
    if not items:
        print("未发现任何 BLE 设备（确认电脑蓝牙已开启）")
        return None
    target = None
    print(f"共 {len(items)} 个设备，含自定义服务 {SERVICE[:8]} 的即为本设备：\n")
    for dev, adv in items:
        uuids = [u.lower() for u in (getattr(adv, "service_uuids", []) or [])]
        name = getattr(adv, "local_name", None) or getattr(dev, "name", None) or "(未命名)"
        hit = any(SERVICE in u for u in uuids) or name == TARGET
        if hit:
            target = dev
            print(f"  ★ {dev.address}  名字={name}  服务={uuids}   <<< 目标设备")
    print("\n--- 其余设备（按名字排序）---")
    for dev, adv in sorted(items, key=lambda x: (getattr(x[1], "local_name", "") or getattr(x[0], "name", "") or "")):
        name = getattr(adv, "local_name", None) or getattr(dev, "name", None) or "(未命名)"
        print(f"     {dev.address}  {name}")
    if not target:
        print(f"\n未发现 {TARGET} / 服务UUID。请确认：设备已上电（单击侧键开机）、未被其他设备连接。")
    return target


async def gatt():
    """枚举设备全部 GATT 服务与特征（禁用 Windows GATT 缓存，避免拿到旧表）。"""
    from bleak import BleakClient, BleakScanner
    print("查找设备 …")
    dev = await BleakScanner.find_device_by_name(TARGET, timeout=12)
    if not dev:
        dev = await BleakScanner.find_device_by_filter(
            lambda d, a: SERVICE in [u.lower() for u in (getattr(a, "service_uuids", []) or [])],
            timeout=12,
        )
    if not dev:
        print(f"未发现 {TARGET}（设备未广播 / 已被别的主机连上）")
        return
    print(f"连接 {dev.address} …")
    async with BleakClient(dev, winrt={"use_cached_services": False}) as cli:
        print(f"已连接  MTU={cli.mtu_size}")
        print("\n=== GATT 枚举 ===")
        for svc in cli.services:
            print(f"\n[服务] {svc.uuid}   handle={getattr(svc, 'handle', '?')}  {svc.description}")
            for ch in svc.characteristics:
                props = ",".join(ch.properties)
                print(f"   [特征] {ch.uuid}  props={props}  {ch.description}")
                for d in ch.descriptors:
                    print(f"        [描述符] {d.uuid}")
        hit = any(svc.uuid.lower() == SERVICE for svc in cli.services)
        print("\n结论：", "自定义服务已注册 ✔" if hit else f"未找到 {SERVICE} ✘（需重新烧录/复位）")


async def probe():
    """端到端联调：订阅状态通知 + 下发 ping/start/stop 命令，验证整条链路。"""
    from bleak import BleakClient, BleakScanner
    print("查找设备 …")
    dev = await BleakScanner.find_device_by_name(TARGET, timeout=12)
    if not dev:
        print(f"未发现 {TARGET}")
        return
    print(f"连接 {dev.address} …")
    buf = ""
    async with BleakClient(dev, winrt={"use_cached_services": False}) as cli:
        def handler(_, data: bytearray):
            nonlocal buf
            buf += data.decode(errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                print("   <<", line.strip())

        await cli.start_notify(CH_STATUS, handler)
        print("已订阅状态通知\n")

        async def send(cmd: str, wait: float = 1.0):
            print(f"   >> {cmd}")
            await cli.write_gatt_char(CH_CMD, cmd.encode(), response=True)
            await asyncio.sleep(wait)

        import time as _t

        await send('{"cmd":"ping"}')
        await send('{"cmd":"tsync","v":%d}' % int(_t.time()))
        await send('{"cmd":"tzsync","v":480}')   # +8 东八区
        await send('{"cmd":"th","v":0.4}')
        await send('{"cmd":"start"}', 5)      # 跑 5 秒
        await send('{"cmd":"stop"}', 1.0)
        await send('{"cmd":"archive"}', 1.0)  # 存档并复位
        await send('{"cmd":"dump"}', 2.5)     # 拉取历史存档
        await cli.stop_notify(CH_STATUS)
        print("\n联调结束：能看到 st=1(运行中) 与 st=2(结束) 即链路正常。")


async def watch(seconds: float = 15.0):
    from bleak import BleakClient, BleakScanner
    print("查找设备 …")
    dev = await BleakScanner.find_device_by_name(TARGET, timeout=10)
    if not dev:
        print(f"未发现 {TARGET}")
        return
    print(f"连接 {dev.address} …")
    buf = ""
    async with BleakClient(dev) as cli:
        def handler(_, data: bytearray):
            nonlocal buf
            buf += data.decode(errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                print("  <<", line.strip())

        await cli.start_notify(CH_STATUS, handler)
        print(f"已订阅状态通知，监听 {seconds:.0f} 秒（按设备按键会看到状态变化）")
        await asyncio.sleep(seconds)
        await cli.stop_notify(CH_STATUS)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "--gatt":
        asyncio.run(gatt())
    elif mode == "--probe":
        asyncio.run(probe())
    elif mode == "--watch":
        asyncio.run(watch())
    else:
        asyncio.run(scan())
