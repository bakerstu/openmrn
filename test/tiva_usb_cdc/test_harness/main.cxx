#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

int main(int argc, char *argv[])
{
    assert(argc == 2 && "invalid arguments");

    int fd = open(argv[1], O_RDWR);

    assert(fd >= 0 && "invalid USB device");

    for (int i = 0; i < 1000; ++i)
    {
        unsigned char buf[1024];
        int bytes_written = 0;
        int bytes_read = 0;
        do
        {
            ssize_t result = write(fd, buf + bytes_written, 1024 - bytes_written);
            assert(result > 0);
            bytes_written += result;
        } while (bytes_written < 1024);
        do
        {
            ssize_t result = read(fd, buf + bytes_read, 1024 - bytes_read);
            assert(result > 0);
            bytes_read += result;
        } while (bytes_read < 1024);
        printf("iteration: %d\n", i);
    }

    printf("success\n");

    return 0;
}
