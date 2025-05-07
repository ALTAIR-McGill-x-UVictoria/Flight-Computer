Import("env")
import os

def patch_radiohead(packagedir, filename, find_text, replace_text):
    filepath = os.path.join(packagedir, filename)
    if os.path.exists(filepath):
        with open(filepath, 'r') as file:
            content = file.read()
        
        if find_text in content:
            content = content.replace(find_text, replace_text)
            with open(filepath, 'w') as file:
                file.write(content)
            print(f"Patched {filename}")

def before_build(source, target, env):
    # Get the RadioHead library path
    lib_dir = env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV")
    radiohead_dirs = [d for d in os.listdir(lib_dir) if "RadioHead" in d]
    
    if not radiohead_dirs:
        print("RadioHead library not found")
        return
    
    radiohead_dir = os.path.join(lib_dir, radiohead_dirs[0])
    print(f"Found RadioHead at {radiohead_dir}")
    
    # Patch RadioHead.h to fix SPI.h include
    find_text = '#include <SPI.h>'
    replace_text = '#if defined(TEENSYDUINO)\n#include "SPI.h"\n#else\n#include <SPI.h>\n#endif'
    patch_radiohead(radiohead_dir, "RadioHead.h", find_text, replace_text)

env.AddPreAction("buildprog", before_build)