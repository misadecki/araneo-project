# Araneo

Quadruped walking robot controlled with micro-ROS as my engineering degree
project.

- **PC side:** ROS 2 Jazzy (gait generator, kinematics, RViz, micro-ROS agent),
running in a distrobox (I use Arch btw).
- **MCU side:** Raspberry Pi Pico 2W (RP2350) with Zephyr 4.1.0 and a micro-ROS client, connected over USB (CDC ACM).
- **Link:** `ROS 2 --/servo_cmd--> micro-ROS agent --USB serial--> Pico 2W`

Wi-Fi on the Pico 2W needs Zephyr >= 4.3, while the micro-ROS module used here works with 4.1, so the robot is connected with a cable for now.

---

## Pinned versions

Version mismatch was the main source of problems, so everything is pinned.

| Component | Version |
|---|---|
| Docker base image | `zephyrprojectrtos/zephyr-build:v0.26.17` |
| Zephyr | `v4.1.0` |
| Zephyr SDK (inside the image) | `0.16.9-rc3` (`/opt/toolchains/zephyr-sdk-0.16.9-rc3`) |
| micro-ROS Zephyr module | branch `jazzy`, commit `87dbe3a` |
| ROS 2 | Jazzy |
| `empy` | `3.3.4` |

## Repository layout

```
araneo-project/
├── Dockerfile
├── firmware/            Zephyr application (micro-ROS client)
│   ├── src, inc, boards, dts, prj.conf, CMakeLists.txt
│   ├── patches/libmicroros-quadruped.patch
│   └── .clangd
├── ros2_ws/             ROS 2 workspace
│   ├── deps.repos       third-party ROS packages (micro-ROS agent)
│   └── src/             gait, bringup, quadruped_msgs, kinematics, ...
├── electronics/
└── mechanics/
```

On the host everything lives under one directory that is mounted into the container:

```
~/microros_ws/
├── araneo-project/      this repository
└── zephyrproject/       Zephyr workspace (not in git)
```

Inside the container this is `/workspace`.

## Where things run

| Where | What | Purpose |
|---|---|---|
| Host | editor, git, SSH keys | editing, commits, flashing |
| Docker container `zephyr-dev` | Zephyr 4.1, SDK, micro-ROS library | **building the firmware only** |
| Distrobox | ROS 2 Jazzy | `gait_node`, RViz, micro-ROS agent |

Do `git clone` and `git push` on the host (the keys are there). Do not copy SSH keys into the container.

---

## 1. Host requirements

- Docker
- distrobox with an Ubuntu image that has ROS 2 Jazzy (if you use Arch as me btw)
- `clangd`, `jq` (optional, for the editor)
- user in the serial group (Arch: `uucp`) to open `/dev/ttyACM*`:
  `sudo usermod -aG uucp $USER` and log in again

```bash
mkdir -p ~/microros_ws
cd ~/microros_ws
git clone <repo-url> araneo-project
```

## 2. Docker image and container

### Dockerfile

The base image does not contain everything that is needed, so the repository has its own `Dockerfile`:

```dockerfile
FROM zephyrprojectrtos/zephyr-build:v0.26.17
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip git cmake build-essential pkg-config libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# build tools required by the micro-ROS library
RUN pip3 install colcon-common-extensions vcstool catkin_pkg lark \
      empy==3.3.4 numpy setuptools

# picotool (flashing from the container - you can use picotool from host)
RUN git clone --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk \
 && git clone --depth 1 https://github.com/raspberrypi/picotool.git /opt/picotool \
 && cmake -S /opt/picotool -B /opt/picotool/build -DPICO_SDK_PATH=/opt/pico-sdk \
 && cmake --build /opt/picotool/build -j"$(nproc)" \
 && cmake --install /opt/picotool/build

# the base image points ZEPHYR_BASE to a directory that does not exist
ENV ZEPHYR_BASE=/workspace/zephyrproject/zephyr

# the workspace is bind-mounted and owned by another user, git would refuse it
RUN git config --system --add safe.directory '*'

WORKDIR /workspace/zephyrproject
CMD ["bash"]
```

### Build the image and create the container (once)

```bash
cd ~/microros_ws/araneo-project
docker build -t araneo-zephyr .

docker run -dit \
  --name zephyr-dev --hostname zephyr-dev \
  -v ~/microros_ws:/workspace \
  -w /workspace/zephyrproject \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' \
  araneo-zephyr
```

- `/dev/bus/usb` + cgroup rule `c 189:*` (USB major number) give `picotool` access without `--privileged`.
- The serial port (`/dev/ttyACM*`) is used on the host, not in the container.

### Daily use

```bash
docker start zephyr-dev            # start (after reboot)
docker exec -it zephyr-dev bash    # enter (any number of terminals)
exit                               # leave, the container keeps running
docker stop zephyr-dev             # stop
```

Everything outside `/workspace` (apt packages, files in `/opt`) lives in the container layer and is lost after `docker rm`. Files in `/workspace` are on the host and are safe.

## 3. Zephyr workspace and micro-ROS module (once, inside the container)

```bash
cd /workspace
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.1.0 zephyrproject
cd zephyrproject
west update                        # several GB, takes a while
west zephyr-export
pip3 install -r zephyr/scripts/requirements.txt

git clone -b jazzy https://github.com/micro-ROS/micro_ros_zephyr_module.git \
    modules/lib/micro_ros_zephyr_module
```

Pin the module to the tested commit and apply the patch that adds `quadruped_msgs` to the micro-ROS library:

```bash
git -C modules/lib/micro_ros_zephyr_module checkout 87dbe3a
git -C modules/lib/micro_ros_zephyr_module apply \
    /workspace/araneo-project/firmware/patches/libmicroros-quadruped.patch
```

### Why a patch for `quadruped_msgs`

This branch of the module has a hard-coded list of packages (`libmicroros.mk`) and does not support `extra_packages`. The patch adds one line to the rule that prepares `micro_ros_src/src`:

```make
cp -r /workspace/araneo-project/ros2_ws/src/quadruped_msgs src/quadruped_msgs; \
```

The path is the one seen **inside the container**. A custom message type must be known on both sides (firmware library and ROS 2 agent), otherwise they do not understand each other.

### Creating or refreshing the patch

Do this after editing `libmicroros.mk` by hand, or if the patch stops applying:

```bash
mkdir -p /workspace/araneo-project/firmware/patches
cd /workspace/zephyrproject/modules/lib/micro_ros_zephyr_module
git diff > /workspace/araneo-project/firmware/patches/libmicroros-quadruped.patch
```

## 4. Build the firmware

Inside the container:

```bash
cd /workspace/zephyrproject
west build -p -b rpi_pico2/rp2350a/m33 /workspace/araneo-project/firmware \
  -d /workspace/araneo-project/firmware/build -- \
  -DZEPHYR_EXTRA_MODULES=/workspace/zephyrproject/modules/lib/micro_ros_zephyr_module/modules/libmicroros \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

- The first build is long (several to a dozen or so minutes): colcon builds the whole micro-ROS library.
- Later builds: drop `-p` (pristine). `-p` wipes the Zephyr build directory, but the micro-ROS library is cached outside it, so it is not rebuilt.

### After changing a `.msg` or any `MICROROS_*` option

The library is cached in the module directory. Force a rebuild (keep `libmicroros.mk`!):

```bash
M=/workspace/zephyrproject/modules/lib/micro_ros_zephyr_module/modules/libmicroros
rm -rf $M/libmicroros.a $M/include $M/micro_ros_src $M/configured_colcon.meta
```

then build again. `micro_ros_src/src` is recreated, so the patched `cp` copies the current `quadruped_msgs`. Check the result on the host:

```bash
ls ~/microros_ws/zephyrproject/modules/lib/micro_ros_zephyr_module/modules/libmicroros/include/quadruped_msgs/msg
# servo_cmd.h  foot_targets.h ...
```

### Configuration notes

- Transport: `CONFIG_MICROROS_TRANSPORT_SERIAL_USB=y` (USB CDC ACM on the old USB stack). It looks for the device named `CDC_ACM_0` (overlay in `firmware/boards`).
- The micro-ROS transport owns the CDC port. Do not use it as console or shell backend, otherwise log text corrupts the protocol frames. Logs go to `uart0` (GPIO 0/1).
- `prj.conf` options such as `CONFIG_PTHREAD_IPC` and `CONFIG_POSIX_CLOCK` exist in Zephyr 4.1 only. Do not build this project with a newer Zephyr (see Troubleshooting).

## 5. Flash the Pico

1. Hold **BOOTSEL**, plug in USB. The Pico appears as a mass-storage drive (`RP2350`).
2. Either copy the file on the host:
   ```bash
   cp ~/microros_ws/araneo-project/firmware/build/zephyr/zephyr.uf2 /run/media/$USER/RP2350/
   ```
   or use picotool in the container:
   ```bash
   picotool load -x /workspace/araneo-project/firmware/build/zephyr/zephyr.uf2
   ```
3. After a reset the Pico enumerates as `/dev/ttyACM0` on the host (`lsusb | grep 2fe3`).

## 6. ROS 2 side (distrobox)

```bash
distrobox enter <name>
cd ~/microros_ws/araneo-project/ros2_ws

vcs import < deps.repos                       # micro-ROS agent sources
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Use the same form of `colcon build` every time (do not switch between `--symlink-install` and a plain build without cleaning).

Run the agent and the robot:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
ros2 launch quadruped_bringup <launch_file>.py          # RViz + agent
```

Checks:

```bash
ros2 node list                    # /araneo_firmware appears after the session is up
ros2 topic info /servo_cmd -v
ros2 run rqt_graph rqt_graph
```

The stock `microros/micro-ros-agent` Docker image does not know `quadruped_msgs`, the agent has to be built together with the workspace.

## 7. Editor setup (host, Neovim + clangd)

The firmware is compiled in the container, so `compile_commands.json` contains container paths. Two things are needed on the host.

### 7.1 Copy the SDK (for system headers such as `newlib.h`)

```bash
docker cp zephyr-dev:/opt/toolchains/zephyr-sdk-0.16.9-rc3 ~/zephyr-sdk-0.16.9-rc3
```

### 7.2 `ksync` alias (run after **every** build)

It fixes ownership of the build directory (created by root in the container) and rewrites the paths:

```bash
ksync() {
  local ws="$HOME/microros_ws"
  local build="$ws/araneo-project/firmware/build"
  sudo chown -R "$USER": "$build"
  sed -i \
    -e "s|/workspace|$ws|g" \
    -e "s|/opt/toolchains/zephyr-sdk-0.16.9-rc3|$HOME/zephyr-sdk-0.16.9-rc3|g" \
    "$build/compile_commands.json"
}
```

Put it in `~/.bashrc` (or `~/.zshrc`). Verify:

```bash
grep -c "/workspace" ~/microros_ws/araneo-project/firmware/build/compile_commands.json   # 0
grep -c "/opt/toolchains" ~/microros_ws/araneo-project/firmware/build/compile_commands.json   # 0
```

### 7.3 `firmware/.clangd`

```yaml
CompileFlags:
  CompilationDatabase: build
  Remove:
    - -mfp16-format*
    - -fno-reorder-functions
    - -fno-defer-pop
    - -fno-printf-return-value
    - -fno-freestanding
    - -specs=*
```

`CompilationDatabase` is relative to the directory of the `.clangd` file, so it is `build`, not `firmware/build`. Do not add `--target` or `-mcpu` here: the database already contains the right flags. If clangd still warns about FPU, add `Add: [--target=arm-none-eabi, -mcpu=cortex-m33+nodsp+nofp, -mfloat-abi=soft]` as the last resort.

### 7.4 `ros2_ws/.clangd`

```yaml
CompileFlags:
  CompilationDatabase: .
  Add: [-std=c++17]
```

Colcon creates one database per package. Merge them after a build:

```bash
cd ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
jq -s 'add' build/*/compile_commands.json > compile_commands.json
```

### 7.5 Neovim

Minimal clangd command (no `--compile-commands-dir`, no `--target`):

```lua
vim.lsp.config('clangd', {
  cmd = {
    'clangd',
    '--query-driver=/home/<user>/zephyr-sdk-0.16.9-rc3/arm-zephyr-eabi/bin/*,/usr/bin/*',
    '--background-index',
  },
})
vim.lsp.enable('clangd')
```

- Firmware: open it in Neovim on the **host**.
- ROS 2 C++ code: open it in Neovim **inside the distrobox** (after `source /opt/ros/jazzy/setup.bash`), because `/opt/ros/jazzy` exists only there.
- Restart clangd after changes (`:lua vim.lsp.enable('clangd', false)` then `:lua vim.lsp.enable('clangd')`) and remove `~/.cache/clangd` if it keeps stale results.
- Do not keep a global `~/.config/clangd/config.yaml` with Zephyr flags; it is added to every project, including ROS 2.

---

## Troubleshooting

### Docker / container

| Symptom | Cause and fix |
|---|---|
| Terminal "hangs" after pasting a long command | An unmatched `'` or `"` (often a leftover from `docker exec ... bash -c '...'`). Press `Ctrl+C` and remove the stray quote. |
| `apt`: `Permission denied` in `/var/lib/apt/lists` | You are not root. Enter with `docker exec -it -u root zephyr-dev bash` (or set `USER root` in the Dockerfile). |
| `docker cp ...: device or resource busy` | The target is a bind mount. Edit the file on the host instead. |
| Files in `~/microros_ws` are read-only in the editor | They are owned by root (created in the container). `sudo chown -R $USER: ~/microros_ws`. |
| `fatal: detected dubious ownership` | `git config --global --add safe.directory '*'` (already set in the Dockerfile). |
| `WARNING: UNPROTECTED PRIVATE KEY FILE` | Keep SSH keys on the host and do `git clone/push` there. Otherwise `chmod 600` the key. |
| Pico not visible in the container | Check `lsusb` on the host and `ls -R /dev/bus/usb` in the container; the container needs `-v /dev/bus/usb:/dev/bus/usb` and the cgroup rule `c 189:* rmw`. |

### Zephyr / west

| Symptom | Cause and fix |
|---|---|
| `west: unknown command "build"` | West was run outside a workspace. Run it from `/workspace/zephyrproject`. |
| `Could not find a west workspace` | `ZEPHYR_BASE` points to a non-existing `/workdir/zephyr`. `export ZEPHYR_BASE=/workspace/zephyrproject/zephyr` (set in the Dockerfile) and make sure `west update` finished. |
| `CMake Error: The source ... does not match the source ... used to generate cache` | The build directory was created for another application. `rm -rf firmware/build`. One build directory per application. |
| `... is not a valid zephyr module` | The module directory is not a Zephyr module. Point `ZEPHYR_EXTRA_MODULES` to `.../modules/libmicroros` (it contains `zephyr/module.yml`), not to the repository root. |
| `colcon: not found` while building the library | `pip3 install colcon-common-extensions` (included in the Dockerfile). |
| `empy` / code generation errors in the micro-ROS build | Wrong `empy` version, it must be `3.3.4`. |
| `quadruped_msgs/msg/servo_cmd.h` not found | The library was not rebuilt after the patch. Apply the patch, delete `libmicroros.a`, `include`, `micro_ros_src`, rebuild (section 4). `extra_packages` is **not** supported by this module version. |
| Last build steps (`[239/254] Linking ...`) are slow | Normal. Do not use `-p` unless needed. Check CPU with `docker stats zephyr-dev`. |
| CMake warnings about `USB_DEVICE_VID/PID` | Harmless for a prototype. |
| Pico does not enumerate as `/dev/ttyACM*` | The transport uses the deprecated USB device stack; check the overlay (`CDC_ACM_0`) and `lsusb`. |
| Agent never creates a session | Logs or shell are enabled on the CDC port. Move the console to `uart0`. |

### ROS 2

| Symptom | Cause and fix |
|---|---|
| `Package 'micro_ros_agent' not found` | Agent is not built or the workspace is
not sourced. `vcs import < deps.repos`, `colcon build --symlink-install`, `source install/setup.bash`. |
| `failed to create symbolic link ... existing path cannot be removed` | Mixed `--symlink-install` and plain builds. `rm -rf build/<pkg> install/<pkg>` and rebuild (or remove `build install log`). |
| `/dev/ttyACM0: Permission denied` | Add the user to `uucp` (Arch) / `dialout` (Debian) on the host and log in again. |
| Pico node missing in `ros2 node list` | The agent was started after the Pico booted and the session was not established. Restart the agent / replug; the client retries with `rmw_uros_ping_agent`. |

### Editor (clangd)

| Symptom | Cause and fix |
|---|---|
| `newlib.h not found` | The SDK is not on the host or `ksync` did not rewrite its path. Copy the SDK (7.1), fix `ksync`, check `--query-driver`. |
| `rcl/rcl.h` / `quadruped_msgs/...` not found | `compile_commands.json` is stale or contains `/workspace`. Rebuild, then `ksync`. For headers check `include/` in `libmicroros`. |
| `processor architecture not supported` | clangd does not use the database ("Generic fallback command" in `clangd --check`). Fix `CompilationDatabase: build` in `firmware/.clangd` and remove the global Zephyr config. |
| ARM flags applied to ROS 2 files (`csignal not found`) | An `.clangd` in the repository root or the global config adds `--target=arm-none-eabi`. Keep separate `.clangd` files in `firmware/` and `ros2_ws/`. |
| `sed: couldn't open temporary file ... Permission denied` | The build directory is owned by root. `ksync` runs `chown` first. |
| FPU warning from `core_cm33.h` | Cosmetic. See 7.3. |
| Debug: what does clangd use? | `clangd --check=src/main.c 2>&1 | grep -i "compile command\|fallback"` |

---

## Notes

- The micro-ROS client runs in its own thread (`microros_client.c`); `main()` only consumes the latest command and drives the servos. If the agent disappears the client destroys its entities (`*_fini`) and creates them again when the agent is back.
- WiFi (UDP transport) is planned after moving to Zephyr >= 4.3. That requires rebuilding the micro-ROS library for that Zephyr version and removing the options that no longer exist (`PTHREAD_IPC`, `POSIX_CLOCK`).
- More info [here](https://github.com/micro-ROS/micro_ros_zephyr_module).
