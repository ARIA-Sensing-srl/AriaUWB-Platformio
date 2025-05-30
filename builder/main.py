# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os

IS_WINDOWS = sys.platform.startswith("win")

from SCons.Script import (
    ARGUMENTS,
    COMMAND_LINE_TARGETS,
    AlwaysBuild,
    Builder,
    Default,
    DefaultEnvironment,
)


def generate_disassembly(target, source, env):
    elf_file = target[0].get_path()
    assert elf_file.endswith(".elf")
    env.Execute(
        " ".join(
            [
                env.subst("$CC").replace("-gcc", "-objdump"),
                "-d",
                '"%s"' % elf_file,
                ">",
                '"%s"' % elf_file.replace(".elf", ".dis"),
            ]
        )
    )


env = DefaultEnvironment()
platform = env.PioPlatform()
board_config = env.BoardConfig()

if IS_WINDOWS==False:
    env.Replace(
        AR="riscv32-unknown-elf-gcc-ar",
        AS="riscv32-unknown-elf-as",
        CC="riscv32-unknown-elf-gcc",
        GDB="riscv32-unknown-elf-gdb",
        CXX="riscv32-unknown-elf-g++",
        OBJCOPY="riscv32-unknown-elf-objcopy",
        RANLIB="riscv32-unknown-elf-gcc-ranlib",
        SIZETOOL="riscv32-unknown-elf-size",
        ARFLAGS=["rc"],
        PROGSUFFIX=".elf",
    )
else:
    env.Replace(
        AR="riscv32-unknown-elf-gcc-ar.bat",
        AS="riscv32-unknown-elf-as.bat",
        CC="riscv32-unknown-elf-gcc.bat",
        GDB="..\\win_bin\\riscv32-unknown-elf-gdb",
        CXX="riscv32-unknown-elf-g++.bat",
        OBJCOPY="riscv32-unknown-elf-objcopy.bat",
        RANLIB="riscv32-unknown-elf-gcc-ranlib.bat",
        SIZETOOL="riscv32-unknown-elf-size.bat",
        ARFLAGS=["rc"],
        PROGSUFFIX=".elf",
    )



# Allow user to override via pre:script
if env.get("PROGNAME", "program") == "program":
    env.Replace(PROGNAME="firmware")

env.Append(
    BUILDERS=dict(
        ElfToHex=Builder(
            action=env.VerboseAction(
                " ".join(["$OBJCOPY", "-O", "ihex", "$SOURCES", "$TARGET"]),
                "Building $TARGET",
            ),
            suffix=".hex",
        ),
        ElfToBin=Builder(
            action=env.VerboseAction(
                " ".join(["$OBJCOPY", "-O", "binary", "$SOURCES", "$TARGET"]),
                "Building $TARGET",
            ),
            suffix=".bin",
        ),
    )
)

pioframework = env.get("PIOFRAMEWORK", [])

if not pioframework:
    env.SConscript("frameworks/_bare.py", exports="env")

#
# Target: Build executable and linkable firmware
#


target_elf = None
if "nobuild" in COMMAND_LINE_TARGETS:
    target_elf = os.path.join("$BUILD_DIR", "${PROGNAME}.elf")
    target_bin = os.path.join("$BUILD_DIR", "${PROGNAME}.bin")
else:
    target_elf = env.BuildProgram()
    target_bin = env.ElfToBin(os.path.join("$BUILD_DIR", "${PROGNAME}"), target_elf)

AlwaysBuild(env.Alias("nobuild", target_bin))
target_buildprog = env.Alias("buildprog", target_bin, target_bin)

env.AddPostAction(
    target_elf, env.VerboseAction(generate_disassembly, "Generating disassembly")
)

#
# Target: Print binary size
#

target_size = env.AddPlatformTarget(
    "size",
    target_elf,
    env.VerboseAction("$SIZEPRINTCMD", "Calculating size $SOURCE"),
    "Program Size",
    "Calculate program size",
)



upload_protocol = env.subst("$UPLOAD_PROTOCOL")
debug_tools = board_config.get("debug.tools", {})
upload_actions = []
upload_target = target_elf

if upload_protocol in debug_tools:
    uploader = "openocd"
    uploader_flags = [
        "-c",
        "debug_level %d" % (2 if int(ARGUMENTS.get("PIOVERBOSE", 0)) else 1),
        "-s",
        platform.get_package_dir("tool-openocd") or "",
    ]
    uploader_flags.extend(
        debug_tools.get(upload_protocol).get("server").get("arguments", [])
    )
    uploader_flags.extend(
        [
            "-c",
            "load_image {$SOURCE} %s"
            % board_config.get("upload").get("image_offset", ""),
            "-c",
            "reset run",
            "-c",
            "shutdown",
        ]
    )

    env.Replace(
        UPLOADER=uploader,
        UPLOADERFLAGS=uploader_flags,
        UPLOADCMD="$UPLOADER $UPLOADERFLAGS",
    )
    upload_actions = [env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")]

# custom upload tool
elif upload_protocol == "custom":
    upload_actions = [env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")]

else:
    sys.stderr.write("Warning! Unknown upload protocol %s\n" % upload_protocol)

env.AddPlatformTarget("upload", upload_target, upload_actions, "Upload")


#
# Setup default targets
#

Default([target_buildprog, target_size])
