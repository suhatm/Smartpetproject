"""僵尸连接测试：连接后不订阅、不读写，模拟被杀的 App，等板子看门狗踢人。"""
import asyncio, sys, time
from bleak import BleakScanner, BleakClient

UUID_SVC = "e5a00001-1e5c-4b8f-9a2d-6c0f7e8d9a0b"

async def main():
    print("扫描目标设备 ...")
    target = None
    for _ in range(8):
        devs = await BleakScanner.discover(timeout=4, return_adv=True)
        for d, adv in devs.values():
            if any(UUID_SVC == (u or "").lower() for u in (adv.service_uuids or [])):
                target = d
                break
        if target:
            break
        await asyncio.sleep(2)
    if not target:
        print("!! 没找到设备（可能仍被占用）"); return 1

    print(f"连接 {target.address}（连接后什么都不做，模拟被杀的 App）...")
    disconnected_evt = asyncio.Event()

    def on_dc(*a):
        disconnected_evt.set()

    async with BleakClient(target.address, disconnected_callback=on_dc) as client:
        t0 = time.time()
        print(f"已连接 {target.address}。挂机等待被踢（最长 160s）...")
        try:
            await asyncio.wait_for(disconnected_evt.wait(), timeout=160)
            print(f"\n★ 板子在 {time.time()-t0:.0f}s 时主动断开了我们 —— 看门狗生效！")
        except asyncio.TimeoutError:
            print(f"\n!! {time.time()-t0:.0f}s 仍未被断开 —— 看门狗未生效")
            return 1
    print("正常退出（连接已由板子断开）")
    return 0

sys.exit(asyncio.run(main()))
