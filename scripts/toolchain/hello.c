/*
 * hello.c — end-to-end proof of the ARM cross-toolchain.
 *
 * Goal: build this statically on the Mac, FTP to /data/video on the drone,
 * chmod +x, run over telnet. If it prints and exits cleanly, the toolchain
 * targets the drone correctly and we can move on to the motorboard driver.
 *
 * It deliberately prints facts that confirm we hit the RIGHT target:
 *   - pointer width  -> must be 4 bytes (32-bit armv7), not 8 (arm64/x86_64)
 *   - the uname()    -> should report Linux 2.6.32 armv7l "mykonos2"
 * Pure libc, no external deps, so a -static build is fully self-contained.
 */
#include <stdio.h>
#include <sys/utsname.h>

int main(void)
{
    struct utsname u;

    printf("hello from a cross-compiled binary\n");
    printf("pointer width: %zu bytes (expect 4 on the drone's armv7)\n",
           sizeof(void *));

    if (uname(&u) == 0) {
        printf("uname: %s %s %s %s\n",
               u.sysname, u.nodename, u.release, u.machine);
    } else {
        printf("uname: (call failed)\n");
    }

    return 0;
}
