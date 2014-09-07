
Build instructions.
============================================================

1. Make sure that #define BL_DEBUG and #define SIMULATE_SWDCLK_PIN_HI
   in config.h are not active (comment them out).

2. Build release build of subproject bootld first.
   Make sure optimize options "High" and "Size" are selected.
   2 warnings from the linker is expected.

3. Convert the binary output from the bootld build to a h-file
   for inclusion in the bootldld sub project:

   Execute the following in the project "src" directory:

   bin2h.exe ../iar/bootld/Release/Exe/bootld.bin ../src/bootld.h -v bootloader

4. Build release build of subproject bootldld.
   Make sure optimize options "High" and "Size" are selected.
   1 warning from the compiler is expected.
   The binary from this build (bootldld.bin) is the actual bootloader.