"""烧录后钩子：StickS3 用原生 USB-Serial/JTAG，esptool 的 RTS 硬复位无效，
烧完会停在 ROM 下载模式。这里在 upload 之后强制用看门狗复位启动应用。
"""
import os
import subprocess
import sys

Import("env")  # noqa: F821  (SCons)


def after_upload(source, target, env):  # noqa: F811
    port = env.GetProjectOption("upload_port", "") or env.GetProjectOption("monitor_port", "")
    if not port:
        print(">>> 未指定 upload_port，跳过烧录后复位")
        return
    esp = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")
    print(f">>> 烧录后看门狗复位 {port}（USB-Serial/JTAG 无法用 RTS 复位）")
    try:
        subprocess.run(
            [sys.executable, esp, "--port", port, "--after", "watchdog_reset", "read_mac"],
            check=False,
        )
    except Exception as e:  # 复位失败不应让整个上传报错
        print(">>> 复位命令执行失败：", e)


env.AddPostAction("upload", after_upload)  # noqa: F821
