# PlatformIO post-script for env c3 / c3_ota: feed the OTA password from the
# gitignored data/config.json to espota, so it never lands in platformio.ini.
import json, os
Import("env")

if env.subst("$UPLOAD_PROTOCOL") == "espota":
    cfg = os.path.join(env.subst("$PROJECT_DIR"), "data", "config.json")
    try:
        with open(cfg) as f:
            pw = json.load(f).get("otaPassword", "")
    except (OSError, ValueError):
        pw = ""
    flags = ["--host_port=3233"]  # pinned so one firewall rule covers it
    if pw:
        flags.append("--auth=" + pw)
    env.Append(UPLOADERFLAGS=flags)
