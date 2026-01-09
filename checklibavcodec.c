// file: print_build.c
#include <libavcodec/version.h>
#include <stdio.h>
#define CALC_FFMPEG_VERSION(a,b,c) ( a<<16 | b<<8 | c )
int main() {
    printf("%d\n", LIBAVCODEC_BUILD);
    # if (LIBAVCODEC_BUILD < CALC_FFMPEG_VERSION(61, 9, 108))
    printf("小于\n");
#endif
    return 0;
}