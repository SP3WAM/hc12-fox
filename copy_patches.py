from os.path import join
import shutil

Import("env")

print("copy_patches.py About to copy the framework patches.")

FRAMEWORK = env.subst("$PIOFRAMEWORK")
PLATFORM = env.subst("$PIOPLATFORM")
# full directory path to the framework
FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-" + FRAMEWORK + PLATFORM)
# full directory path to the patches
PATCHES_DIR = join(env.subst("$PROJECT_DIR"), "patches-fw")

print("copy_patches.py Copying files from %s to %s" % (PATCHES_DIR, FRAMEWORK_DIR) )

shutil.copytree(PATCHES_DIR, FRAMEWORK_DIR, dirs_exist_ok=True)
