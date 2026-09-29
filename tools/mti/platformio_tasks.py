"""PlatformIO maintenance targets; never upload or calibrate during build."""
Import("env")
import os
import subprocess

env.BuildSources("$BUILD_DIR/mti", "$PROJECT_DIR/tools/mti", src_filter="+<bridge.cpp>")

def action(mode):
    def run(source, target, env):
        cmd = [env.subst("$PYTHONEXE"), "-u",
               os.path.join(env.subst("$PROJECT_DIR"), "tools", "mti", "calibrate.py"), mode]
        port = env.GetProjectOption("upload_port", None)
        if port:
            cmd += ["--port", port]
        return subprocess.call(cmd)
    return run

env.AddCustomTarget("mti_check", None, action("check"),
                    title="MTi: check measurements", description="Requires bridge firmware; no alignment changes")
env.AddCustomTarget("mti_zero", None, action("zero"),
                    title="MTi: inclination zero + store", description="Stationary, level vehicle and HV OFF required")
