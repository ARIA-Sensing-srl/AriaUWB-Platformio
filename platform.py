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

import os
import sys

from platformio.public import PlatformBase

IS_WINDOWS = sys.platform.startswith("win")

class AriauwbplatformioPlatform(PlatformBase):

    def get_boards(self, id_=None):
        result = super().get_boards(id_)
        if not result:
            return result
        if id_:
            return self._add_default_debug_tools(result)
        else:
            for key in result:
                result[key] = self._add_default_debug_tools(result[key])
        return result

    def _add_default_debug_tools(self, board):
        debug = board.manifest.get("debug", {})
        if "tools" not in debug:
            debug["tools"] = {}

        tools = (
            "ftdijtag",
            "openocd"
        )
        for tool in tools:
            if tool in debug["tools"]:
                continue
            if IS_WINDOWS==False:
                debug["tools"][tool] = {
                    "init_cmds": [
                        "define pio_reset_halt_target",
                        "   load",
                        "   monitor reset halt",
                        "end",
                        "define pio_reset_run_target",
                        "   load",
                        "   monitor reset",
                        "end",
                        "set mem inaccessible-by-default off",
                        "set arch riscv:rv32",
                        "set remotetimeout 250",
                        "target extended-remote $DEBUG_PORT",
                        "$INIT_BREAK",
                        "$LOAD_CMDS",
                        "set $mstatus=0x0",
                        "set *((unsigned int*)0x1a104100) = 0",
                        "set $pc=0x1C000080"
                    ],
                    "server": {
                        "package": "tool-openocd",
                        "executable": "bin/openocd",
                        "arguments": [
                            "-s",
                            os.path.join(self.get_dir(), "misc", "openocd"),
                            "-s",
                            "$PACKAGE_DIR/share/openocd/scripts",
                            "-f",
                            "openocd-1core.cfg",
                        ]
                    },
                    "onboard": tool in debug.get("onboard_tools", []),
                }
            else:
                debug["tools"][tool] = {
                    "init_cmds": [
                        "define pio_reset_halt_target",
                        "   load",
                        "   monitor reset halt",
                        "end",
                        "define pio_reset_run_target",
                        "   load",
                        "   monitor reset",
                        "end",
                        "set mem inaccessible-by-default off",
                        "set arch riscv:rv32",
                        "set substitute-path /mnt/c C:"
                        "set remotetimeout 250",
                        "target extended-remote $DEBUG_PORT",
                        "$INIT_BREAK",
                        "$LOAD_CMDS",
                        "set $mstatus=0x0",
                        "set *((unsigned int*)0x1a104100) = 0",
                        "set $pc=0x1C000080"
                    ],
                    "server": {
                        "package": "tool-openocd",
                        "executable": "bin/openocd",
                        "arguments": [
                            "-s",
                            os.path.join(self.get_dir(), "misc", "openocd"),
                            "-s",
                            "$PACKAGE_DIR/share/openocd/scripts",
                            "-f",
                            "openocd-1core.cfg",
                        ]
                    },
                    "onboard": tool in debug.get("onboard_tools", []),
                }

        board.manifest["debug"] = debug
        return board
