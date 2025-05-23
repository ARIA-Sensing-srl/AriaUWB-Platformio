from os.path import isdir, isfile, join, dirname, realpath
from string import Template
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()

# import default build settings
env.SConscript("_bare.py")

FRAMEWORK_DIR = platform.get_package_dir("ARIAUWB-pio-framework-hydrogen")
assert isdir(FRAMEWORK_DIR)



def get_flag_value(flag_name:str, default_val:bool):
    flag_val = board.get("build.%s" % flag_name, default_val)
    flag_val = str(flag_val).lower() in ("1", "yes", "true")
    return flag_val

# the linker script also uses $ on it, so we can't use that as the
# variable identifier for the substitution engine.
class CustomTemplate(Template):
	delimiter = "#"

env.Append(
    CPPPATH=[
        join(FRAMEWORK_DIR, "common"),
        join(FRAMEWORK_DIR, "comunication"),
        join(FRAMEWORK_DIR, "comunication_SPIS"),
        join(FRAMEWORK_DIR, "driverUser"),
        join(FRAMEWORK_DIR, "hal/inc"),
        join(FRAMEWORK_DIR, "hw"),
        join(FRAMEWORK_DIR, "ipc"),
        join(FRAMEWORK_DIR, "coprbin"),
        join(FRAMEWORK_DIR, "processing"),
        join(FRAMEWORK_DIR, "reconstruction"),
        join(FRAMEWORK_DIR, "startup"),
        join(FRAMEWORK_DIR, "utils")
    ]
)

env.Append(
#    ASPPFLAGS=[
#        "-DLANGUAGE_ASSEMBLY",
#        ("-include", os.path.join("$BUILD_DIR", "configs", "sdk", "fc_config.h")),
#    ],
    CCFLAGS=[
    ],
    LINKFLAGS=[
        "-nostartfiles",
        "-Wl,-melf32lriscv",
        "-Wl,--gc-sections",
        "--specs=nosys.specs",
        "--specs=nano.specs"
    ],
)

env.Replace(
    LDSCRIPT_PATH=join(FRAMEWORK_DIR,"LD/link_hydr_cv32e40_wcopr.ld"))

libs = []

env.BuildSources(
    join("$BUILD_DIR", "comunication"),
    join(FRAMEWORK_DIR, "comunication"),
)

env.BuildSources(
    join("$BUILD_DIR", "comunication_SPIS"),
    join(FRAMEWORK_DIR, "comunication_SPIS"),
)

env.BuildSources(
    join("$BUILD_DIR", "driverUser"),
    join(FRAMEWORK_DIR, "driverUser"),
)

env.BuildSources(
    join("$BUILD_DIR", "hal"),
    join(FRAMEWORK_DIR, "hal/src"),
)

env.BuildSources(
    join("$BUILD_DIR", "ipc"),
    join(FRAMEWORK_DIR, "ipc"),
)

env.BuildSources(
    join("$BUILD_DIR", "processing"),
    join(FRAMEWORK_DIR, "processing"),
)

env.BuildSources(
    join("$BUILD_DIR", "reconstruction"),
    join(FRAMEWORK_DIR, "reconstruction"),
)

env.BuildSources(
    join("$BUILD_DIR", "startup"),
    join(FRAMEWORK_DIR, "startup"),
)

env.BuildSources(
    join("$BUILD_DIR", "utils"),
    join(FRAMEWORK_DIR, "utils"),
)

env.Append(LIBPATH=[join(FRAMEWORK_DIR, "hal/libs")])

# mandatory for compilation
libs += ["hal_libs_cv32e40_priv_periph"]

env.Append(LIBS=libs)
